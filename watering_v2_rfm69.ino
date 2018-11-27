/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG 

// Enable and select radio type attached
//#define MY_RADIO_NRF24
#define MY_RADIO_RFM69
//#define MY_IS_RFM69HW
//#define MY_REPEATER_FEATURE
//#define MY_NODE_ID 2

#include <MySensors.h> 
#include <avr/wdt.h>
#include <SPI.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
#include "RTClib.h"

I2CSoilMoistureSensor sensor;
RTC_DS3231 rtc;

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//Sketch information
#define SKETCH_INFO       "Watering controller"
#define SKETCH_VERSION    "0.1"
#define SOIL_ID_INFO      "Soil Water Content"
#define SOIL_ID 0
#define TEMP_ID_INFO      "Ground Sensor Temperature"
#define TEMP_ID 1
#define BVOLT_ID_INFO     "Battery Voltage"
#define BVOLT_ID 2
#define SVOLT_ID_INFO     "Solar Voltage"
#define SVOLT_ID 3
#define RADIOTEMP_ID_INFO "Radio Temperature"
#define RADIOTEMP_ID 4
#define RSSI_ID_INFO      "Radio RSSI"
#define RSSI_ID 5

//for 1xli-ion
#define VMIN 3.5
#define VMAX 4.1  
#define VDELTA 0.6
#define BATT_CALC 0.0061070412
#define SOLAR_CALC 0.0117397661

#define BATTERY_READS 10
#define BATTERY_SENSE A3
#define SOLAR_SENSE A0

//External Watchdog heartbeat
#define PULSEPIN A1

//Soil Sensor VCC pin
#define SOILVCCPIN 4

//Auto-reset
#define MESSAGES_FAILED_REBOOT 20
#define CICLES_REBOOT 2880

//RSSI calculations
#define RSSI_READS 3

//Configurable ACK Timeout
#define ACK_TIMEOUT 4000

//BAUD RATE
#define BAUD_RATE 115200

//Cycles in between updates
#define SECONDS_PER_CICLE 30
#define CICLES_PER_PRESENT 2880
#define CICLES_PER_UPDATE 10

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 10
#define LOW_PRIORITY_RETRIES 2


float lastTemperature=-127,temperature=-127,deltatemp,radioTemperature;
float Sbatt, Vbatt, CurrValue, current;
unsigned int BattValue, Batt, Battarray[BATTERY_READS], Battindex = 0, BattarrayTotal = 0;
int radioRSSIarray[RSSI_READS], radioRSSIindex=0,radioRSSIarrayTotal=0,messagesFailed=0;
int SoilHum;
volatile int radioRSSI, isACKed = false;
unsigned int nosend = CICLES_PER_UPDATE, i;
unsigned int topresent = CICLES_PER_PRESENT;

int dryValue = 373, wetValue = 675, friendlyDryValue = 0, friendlyWetValue = 100, rawSensor;

boolean ack = true;
boolean metric, timeReceived = false; 
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime now;

unsigned long cicles=0;

//Mysensors Messages initialization
MyMessage message;

void before()  
{
  //disable watchdog timer
  MCUSR = 0;
  pinMode(PULSEPIN, INPUT);
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));

  //Setup power level
  //_radio.setPowerLevel(30);
  
  //Setup pin modes for proper operation
  pinMode(SOILVCCPIN, INPUT);
  //blink LED on power up
  pinMode(13,OUTPUT);
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  Serial.println(F("initialing battery voltage"));
  analogReference(INTERNAL);
  analogRead(BATTERY_SENSE);
  for (i = 0; i < BATTERY_READS; i++) {
    analogRead(BATTERY_SENSE);
    Battarray[i] = analogRead(BATTERY_SENSE);
    BattarrayTotal += Battarray[i];
    Serial.print(" ");
    Serial.print((float)Battarray[i] * BATT_CALC);
  }
  Serial.println(F(""));
  Serial.print(F("Battery average "));
  Serial.print((float)BattarrayTotal/BATTERY_READS * BATT_CALC);
  Serial.println(F(""));

  Serial.println(F("initialing RSSI array"));
  for (i = 0; i < RSSI_READS; i++) {
    radioRSSIarray[i] = 0;
  }
}

void setup()  
{
  Serial.println("");
  //activate watchdog timer
  wdt_enable(WDTO_8S);
  
//  metric = getConfig().isMetric;
  metric = true;
  Serial.print(F("Metric: "));
  Serial.println(metric);

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, requesting time!"));
    i=0;
    while (timeReceived != true) {
      Serial.println(F("Requesting Time"));
      //request time
      requestTime(); 
      heartbeat ();
      wait(5000);
      if (i>4) {
        break;
      } else {
        i++;
      }
    }  
  }
  Serial.println(F("Reading Time ftom RTC"));
  now = rtc.now();
  PrintTime (); 
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  now = rtc.now();
  
  heartbeat();
  //check if it should reboot itself
  if ((now.hour() == 2) && (cicles > 12)) {
    Serial.println(F("Reboot hour reached (2)"));
    asm volatile ( "jmp 0");
    wait(100);
  }

  readSoil ();
  readSolar();  
  readBattery();
  readRadioTemp();
  sendValues();

  cicles++;
  Serial.print(F("Waiting in cicle "));
  //displays the current cicle count
  Serial.println(cicles);
  wdsleep(SECONDS_PER_CICLE*1000);
} 

void wake_sensors () {
  //Wake the soil sensor
  Serial.println(F("waking sensor"));
  pinMode(SOILVCCPIN, OUTPUT);
  digitalWrite (SOILVCCPIN, HIGH);
  sleep(100);
  sensor.begin(); // reset sensor
  wdsleep(1000);    // give some time to boot up  
}

void sleep_sensors () {
  //Sleep the temperature sensor
  Serial.println(F("Sleeping sensor"));
  pinMode(SOILVCCPIN, INPUT);
}

void readRadioTemp () {
  radioTemperature = _radio.readTemperature(0);
}

void readSoil () {
  // power up sensor
  wake_sensors ();

  Serial.print("Soil Moisture Capacitance: ");
  rawSensor = sensor.getCapacitance();
  SoilHum = map(rawSensor, dryValue, wetValue, friendlyDryValue, friendlyWetValue);
  Serial.print(rawSensor);
  Serial.print(" Soil Moisture Percentage: ");
  Serial.print(SoilHum);
  Serial.print(", Temperature: ");
  temperature = (sensor.getTemperature()/(float)10);
  Serial.println(temperature); //temperature register
  //sensor.sleep(); // available since FW 2.3
  sleep_sensors();
  
  //print Temperature
  Serial.print(F(" temperature: "));
  Serial.println(temperature, 3);

}

// Reads Battery voltage while averaging the last BATTERY_READS values
void readBattery () {
  
  Serial.println(F("Reading Battery Voltage"));
  //primes the analog converter
  analogRead(BATTERY_SENSE);
  analogRead(BATTERY_SENSE);
  analogRead(BATTERY_SENSE);
  //Reads battery voltage
  Battarray[Battindex] = analogRead(BATTERY_SENSE);
  Serial.print(F("Battery read "));
  Serial.println((float)Battarray[Battindex] * BATT_CALC);
  Serial.print(F("Index "));
  Serial.println(Battindex);
  //calculates the current average
  BattarrayTotal=0;  
  for (i = 0; i < BATTERY_READS; i++) {
    BattarrayTotal += Battarray[i];
  }
  //updates battery index
  Battindex ++;
  if (Battindex >= BATTERY_READS) {
    Battindex = 0;
  }

  BattValue = BattarrayTotal / BATTERY_READS;
  Serial.print(F("Battery analog average "));
  Serial.println(BattValue);
  Serial.print(F("Battery average "));
  Serial.println((float)BattValue * BATT_CALC);

  Vbatt = (float)BattValue * BATT_CALC;
  Batt = (Vbatt - VMIN) * 100.0 / (VDELTA);
  if (Vbatt > VMAX) {
    Batt = 100;
  } else if (Vbatt < VMIN) {
    Batt = 0;
  }

  //print battery status
  Serial.print(F("battery: "));
  Serial.print(Batt);
  Serial.print(F(", "));
  Serial.println(Vbatt, 3);
}

void PrintTime () {
  now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

// Reads Battery voltage while averaging the last value
void readSolar () {
  Serial.println(F("Reading Solar Voltage"));
  //primes the analog converter
  analogRead(SOLAR_SENSE);
  analogRead(SOLAR_SENSE);
  analogRead(SOLAR_SENSE);
  //Reads battery voltage
  BattValue = analogRead(SOLAR_SENSE);
  //adds the current reading
  BattValue = (BattValue + analogRead(SOLAR_SENSE)) / 2;
  Serial.print(F("Solar analog Average "));
  Serial.println(BattValue);
  
  Sbatt = (float)BattValue * SOLAR_CALC;
  
  //print Solar status
  Serial.print(F("Solar voltage: "));
  Serial.println(Sbatt, 3);
}

void sendValues () {

  Serial.print(F("Soil Water Content "));
  Serial.println (SoilHum);
  Serial.print(F(" Temperature "));
  Serial.print(temperature,3); 
  Serial.print(F(" Radio Temperature "));
  Serial.print(radioTemperature);
  Serial.print(F(" Radio RSSI "));
  Serial.println(radioRSSI);
  Serial.print(F("Battery Voltage "));
  Serial.print(Vbatt,3); 
  Serial.print(F(" Solar Voltage "));
  Serial.println(Sbatt,3);

  
  //only sends values if they have changed or if it didn't send for 12 cycles (1 hour)
  if (nosend < CICLES_PER_UPDATE) {
    nosend++;
    return;
  }
  //reset count;
  nosend = 1;

  //print debug message
  Serial.println(F("Sending Values"));
  resend(message.setSensor(SOIL_ID).setType(V_LEVEL).set(SoilHum,0),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  resend(message.setSensor(TEMP_ID).setType(V_TEMP).set(temperature,3),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  resend(message.setSensor(BVOLT_ID).setType(V_VOLTAGE).set(Vbatt,3),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  resend(message.setSensor(SVOLT_ID).setType(V_VOLTAGE).set(Sbatt,3),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  sendBatteryLevel(Batt, ack);
  resend(message.setSensor(RADIOTEMP_ID).setType(V_TEMP).set(radioTemperature,3),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  resend(message.setSensor(RSSI_ID).setType(V_VOLTAGE).set(radioRSSI,0),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
}

void gwPresent () {
  //present at beggining and every day
  if (topresent < CICLES_PER_PRESENT) {
    topresent++;
    return;
  }
  Serial.println(F("Presenting"));
  //reset count;
  topresent = 0;

  sendSketchInfo(SKETCH_INFO, SKETCH_VERSION);
  wait(1000);
  heartbeat();
  present(SOIL_ID, S_MOISTURE, SOIL_ID_INFO);
  wait(1000);
  heartbeat();
  present(TEMP_ID, S_TEMP, TEMP_ID_INFO);
  wait(1000);
  heartbeat();
  present(BVOLT_ID, S_MULTIMETER, BVOLT_ID_INFO);
  wait(1000);
  heartbeat();
  present(SVOLT_ID, S_MULTIMETER, SVOLT_ID_INFO);
  wait(1000);
  heartbeat();
  present(RADIOTEMP_ID, S_TEMP, RADIOTEMP_ID_INFO);
  wait(1000);
  heartbeat();
  present(RSSI_ID, S_MULTIMETER, RSSI_ID_INFO);
  wait(1000);
  heartbeat();
}


void resend(MyMessage &msg, int repeats, int timeout)
{
  int repeat = 0;
  int repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    send(msg,true);

    if (waitACK(timeout)) {
      sendOK = true;
      messagesFailed = 0;
    } else {
      sendOK = false;
      Serial.print("Retry ");
      Serial.print(repeat);
      Serial.print(" Failed ");
      Serial.println(messagesFailed);
      repeatdelay += 500;
      wdsleep(repeatdelay);
    }
    repeat++; 
  }
  if (sendOK == false) {
    if (messagesFailed > MESSAGES_FAILED_REBOOT) {
      asm volatile ( "jmp 0");
    }
    messagesFailed++;
  }
}


boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == true) {
      isACKed = false;
      Serial.print(F("Reply "));
      Serial.print(timeout);
      Serial.print(" ");
      Serial.println((millis() - startTime));
      return true;
    }
  }
  return false;
}

void receive (const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  //Substract the last reading
  radioRSSIarrayTotal = radioRSSIarrayTotal - radioRSSIarray[radioRSSIindex];
  //Replace last reading by current reading
  radioRSSIarray[radioRSSIindex] = _radio.RSSI;
  //Update last reading
  radioRSSIarrayTotal = radioRSSIarrayTotal + radioRSSIarray[radioRSSIindex];
  //Calculates new RSSI array index
  radioRSSIindex ++;
  if (radioRSSIindex >= RSSI_READS) {
    radioRSSIindex = 0;
  }
  //updates radio RSSI
  radioRSSI = radioRSSIarrayTotal / RSSI_READS;
  
  if (message.isAck()) {
    Serial.print(F("This is an ack from gateway. RSSI "));
    Serial.println(_radio.RSSI);
    isACKed = true;
  } else {
    Serial.print(F("Incoming change for sensor:"));
    Serial.print(message.sensor);
    Serial.print(F(" RSSI "));
    Serial.println(_radio.RSSI);
  }
}

void receiveTime(unsigned long time) {
  rtc.adjust(time);
  Serial.println(F("Time Received, RTC updated"));
  timeReceived = true;
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
    wait(90);
    heartbeat();
  }
  #else
    smartSleep(ms);
  #endif
}

void heartbeat () {
  wdt_reset();
  pinMode(PULSEPIN, OUTPUT);
  digitalWrite(PULSEPIN, LOW);
  wait(1);
  // Return to high-Z
  pinMode(PULSEPIN, INPUT);
}

