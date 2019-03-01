/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG 
#define MY_SMART_SLEEP_WAIT_DURATION_MS 10000

// Enable and select radio type attached
//#define MY_RADIO_NRF24
#define MY_RADIO_RFM69
//#define MY_IS_RFM69HW
//#define MY_REPEATER_FEATURE
//#define MY_NODE_ID 2
//#define MY_RFM69_ATC_MODE_DISABLED
//#define MY_RFM69_TX_POWER_DBM 20

#include <MySensors.h> 
#include <avr/wdt.h>
#include <SPI.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
#include "RTClib.h"

I2CSoilMoistureSensor sensor;
RTC_DS3231 rtc;
#define HAS_RTC false

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
#define WATERING_ID_INFO  "Watering Time"
#define WATERING_ID 6
#define DURATION_ID_INFO  "Watering Duration"
#define DURATION_ID 7
#define MANUAL_ID_INFO    "Manual Start"
#define MANUAL_ID 8

//for 1xli-ion
#define VMIN 3.4
#define VMAX 4.0  
#define VDELTA 0.6
//sensor1
//#define BATT_CALC 0.01172352941
//#define SOLAR_CALC 0.0061652892561983
//sensor2
#define BATT_CALC 0.1154285714285714
#define SOLAR_CALC 0.1129268292682927

#define BATTERY_READS 10
//sensor1
//#define BATTERY_SENSE A3
//#define SOLAR_SENSE A0
//sensor2
#define BATTERY_SENSE A6
#define SOLAR_SENSE A7

//External Watchdog heartbeat
//sensor1
//#define PULSEPIN 3
//sensor2
#define PULSEPIN 5
//Soil Sensor VCC pin
#define SOILVCCPIN 4
//Enable Driver pin
#define DRIVERPIN 6
//Set pin
#define SETPIN 7
//Unset pin
#define UNSETPIN 8

//Auto-reset
#define MESSAGES_FAILED_REBOOT 20
#define CICLES_REBOOT 2880

//RSSI calculations
#define RSSI_READS 3

//Configurable ACK Timeout
#define ACK_TIMEOUT 4000

//BAUD RATE
#define BAUD_RATE 115200

//Cicles in between updates
#define SECONDS_PER_CICLE 60
#define CICLES_PER_PRESENT 2880
#define CICLES_PER_UPDATE 5

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 10
#define LOW_PRIORITY_RETRIES 2


float lastTemperature=-127,temperature=-127,deltatemp,radioTemperature;
float Sbatt, Vbatt, CurrValue, current;
unsigned int BattValue, Batt, Battarray[BATTERY_READS], Battindex = 0, BattarrayTotal = 0;
int Start1Min, Start1Hour, Start2Min, Start2Hour,_Start1Min, _Start1Hour, _Start2Min, _Start2Hour, WaterDuration, _WaterDuration, _SoilHumTarget, SoilHumTarget;
int SleepTime;
int radioRSSIarray[RSSI_READS], radioRSSIindex=0,radioRSSIarrayTotal=0,messagesFailed=0;
volatile int radioRSSI, isACKed = false, Manual_request = false, WateringOn = false;
unsigned int nosend = CICLES_PER_UPDATE, i;
unsigned int topresent = CICLES_PER_PRESENT;

//seensor1
//int dryValue = 373, wetValue = 675, friendlyDryValue = 0, friendlyWetValue = 100, rawSensor, SoilHum;
//sensor2
int dryValue = 360, wetValue = 624, friendlyDryValue = 0, friendlyWetValue = 100, rawSensor, SoilHum;

boolean ack = true;
boolean metric, timeReceived = false, Start1 = false, Start2 = false; 
char StartTime[MAX_MESSAGE_LENGTH];
char WaterTime[MAX_MESSAGE_LENGTH];
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
DateTime now,oldnow,water_unset;

unsigned long cicles=0;

//Mysensors Messages initialization
MyMessage message;

void before()  
{
  SleepTime = SECONDS_PER_CICLE;
  //disable watchdog timer
  MCUSR = 0;
  pinMode(PULSEPIN, INPUT);
  pinMode(DRIVERPIN, OUTPUT);
  digitalWrite(DRIVERPIN, LOW);
  pinMode(SETPIN, OUTPUT);
  digitalWrite(SETPIN, LOW);
  pinMode(UNSETPIN, OUTPUT);
  digitalWrite(UNSETPIN, LOW);
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));
  
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

  StartDriver();
  Serial.println(F("Unset Water Relay"));
  digitalWrite (UNSETPIN, HIGH);
  wait (400);
  digitalWrite (UNSETPIN, LOW);
  StopDriver();

  rtc.begin();
}

void setup()  
{
  Serial.println("");
  //activate watchdog timer
  //wdt_enable(WDTO_8S);
  
//  metric = getConfig().isMetric;
  metric = true;
  Serial.print(F("Metric: "));
  Serial.println(metric);

  getTime ();

  PrintTime (now); 

  Serial.println(F("Requesting Watering details"));  
  request(WATERING_ID, V_TEXT, 0);
  wait (5000);
  request(DURATION_ID, V_TEXT, 0);
  wait (500);

  //configures water unset
  water_unset = now;
} 

//Makes sure that the time is updated
void getTime () {
  if (!HAS_RTC || rtc.lostPower()) {
    Serial.println(F("RTC lost power/not present, requesting time!"));
    i=0;
    timeReceived=false;
    while (timeReceived != true) {
      Serial.println(F("Requesting Time"));
      //request time
      requestTime(); 
      heartbeat ();
      wait(2000);
      Serial.print(F("."));
      if (i>10) {
        break;
      } else {
        i++;
      }
    }
    if (HAS_RTC) {
      Serial.println(F("Reading Time from RTC"));
      now = rtc.now();
    } 
  }
  //initialize old time
  oldnow = now;
}

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  //request for commands queuing during sleep
  heartbeat();
  sleep((MY_SMART_SLEEP_WAIT_DURATION_MS+1),true);
  heartbeat();
  
  getTime ();
  
  heartbeat();
  //check if it should reboot itself
  if ((now.hour() == 2) && (cicles > 12)) {
    Serial.println(F("Reboot hour reached (2)"));
    asm volatile ( "jmp 0");
    wait(100);
  }

  //detects end of watering
  Serial.println(F("Time now"));
  PrintTime (now);
  Serial.println(F("Time for relay reset"));
  PrintTime (water_unset);

  //process water logic, read water content
  Watering ();

  //check if water time has passed
  if ((water_unset.secondstime() < now.secondstime()) && (WateringOn == true)) {
    Serial.println(F("Watering time elapsed"));
    Manual_request = false;
    WaterStop();
  }

  //only reads values if we didn't send for CICLES_PER_UPDATE CICLES
  if (nosend < CICLES_PER_UPDATE) {
    nosend++;
  } else {
    //reset count;
    nosend = 1;
  
    readSolar();  
    readBattery();
    readRadioTemp();
    sendValues();
  }

  cicles++;
  Serial.print(F("Waiting in cicle "));
  //displays the current cicle count
  Serial.println(cicles);
  oldnow = now;
  wdsleep((long)SleepTime*1000);
} 

void StartDriver () {
  Serial.println(F("Starting 12v driver"));
  digitalWrite (DRIVERPIN, HIGH);
  wait (2000);
}
void StopDriver () {
  Serial.println(F("Stopping 12v driver"));
  digitalWrite (DRIVERPIN, LOW);  
}
void StartWatering () {
  Serial.println(F("Set Water Relay"));
  resend(message.setSensor(MANUAL_ID).setType(V_ARMED).set(true),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  digitalWrite (SETPIN, HIGH);
  WateringOn = true;
  wait (800);
  digitalWrite (SETPIN, LOW);
}
void StopWatering () {
  Serial.println(F("Unset Water Relay"));
  resend(message.setSensor(MANUAL_ID).setType(V_ARMED).set(false),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  digitalWrite (UNSETPIN, HIGH);
  WateringOn = false;
  wait (400);
  digitalWrite (UNSETPIN, LOW);
}

void WaterStop () {
  StartDriver();
  StopWatering();
  StopDriver();
  Serial.println(F("Watering Stop"));
  //Change SleepTime to original
  SleepTime = SECONDS_PER_CICLE;

}

void WaterStart () {
  Serial.println(F("Watering Start"));
  //Change SleepTime to 30 seconds
  SleepTime = 30;
  StartDriver ();
  StartWatering();
  StopDriver();
}


void Watering () {

  readSoil (); //Read water content
  if ((SoilHum > SoilHumTarget) && (Manual_request == false)) {
    Serial.println(F("Soil Humidity Excedeed"));
    //checks soil humidity if watering, so it can be stoped if excedeed
    if (WateringOn == true) {
      WaterStop();
    }
    return;
  }
  //checks if the water is off but there is a manual start request
  if ((Manual_request == true) && (WateringOn == false)) {
    Serial.println(F("Manual watering start until"));
    water_unset = now + TimeSpan (0,0,WaterDuration,0);
    PrintTime (water_unset);
    WaterStart ();  
  }

  Serial.print(F("WateringOn "));
  Serial.println(WateringOn);
  Serial.print(F("Start1 "));
  Serial.println(Start1);
  Serial.print(F("Start2 "));
  Serial.println(Start2);

  //checks if the first start time has passed
  if ((now.hour() >= Start1Hour) && (now.minute() >= Start1Min) && (Start1 == false) && (SoilHum <= SoilHumTarget)) {
    Serial.println(F("Program 1 watering start"));
    Start1 = true;
    water_unset = now + TimeSpan (0,0,WaterDuration,0);
    PrintTime (water_unset);
    WaterStart ();      
  }

  //checks if the second start time has passed
  if ((now.hour() >= Start2Hour) && (now.minute() >= Start2Min) && (Start2 == false) && (SoilHum <= SoilHumTarget)) {
    Serial.println(F("Program 2 watering start until"));
    Start2 = true;
    water_unset = now + TimeSpan (0,0,WaterDuration,0);
    PrintTime (water_unset);
    WaterStart ();      
  }

  //the day changed, reset timers
  if (now.day() != oldnow.day()) {
    Start1 = false;
    Start2 = false;
  }
}


void wake_sensors () {
  //Wake the soil sensor
  Serial.println(F("waking sensor"));
  pinMode(SOILVCCPIN, OUTPUT);
  digitalWrite (SOILVCCPIN, HIGH);
  wait(500);
  sensor.begin(); // reset sensor
  wait(1000);    // give some time to boot up  
}

void sleep_sensors () {
  //Sleep the temperature sensor
  Serial.println(F("Sleeping sensor"));
  digitalWrite (SOILVCCPIN, LOW);
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

void PrintTime (DateTime time ) {

  Serial.print(time.year(), DEC);
  Serial.print('/');
  Serial.print(time.month(), DEC);
  Serial.print('/');
  Serial.print(time.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[time.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(time.hour(), DEC);
  Serial.print(':');
  Serial.print(time.minute(), DEC);
  Serial.print(':');
  Serial.print(time.second(), DEC);
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

  //print debug message
  Serial.println(F("Sending Values"));
  resend(message.setSensor(SOIL_ID).setType(V_HUM).set(SoilHum,0),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
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
  present(SOIL_ID, S_HUM, SOIL_ID_INFO);
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
 present(WATERING_ID, S_INFO, WATERING_ID_INFO);
  wait(1000);
  heartbeat();
 present(DURATION_ID, S_INFO, DURATION_ID_INFO);
  wait(1000);
  heartbeat();  
 present(MANUAL_ID, S_DOOR, MANUAL_ID_INFO);
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
    if (message.sensor == WATERING_ID) {
      strncpy(StartTime, message.data, MAX_MESSAGE_LENGTH);
      char* token = strtok(StartTime, ":");
      _Start1Hour = atoi(token);
      token = strtok(NULL, "-");
      _Start1Min = atoi(token);
      token = strtok(NULL, ":");
      _Start2Hour = atoi(token);
      token = strtok(NULL, "");
      _Start2Min = atoi(token);
      if ((_Start2Min < 60) && (_Start2Min >= 0) && (_Start1Min < 60) && (_Start1Min >= 0) && (_Start1Hour < 24) && (_Start1Hour >= 0) && (_Start2Hour < 24) && (_Start2Hour >= 0)) {
        Serial.print(F("Received watering start "));
        Serial.print(_Start1Hour);
        Serial.print(F(":"));
        Serial.print(_Start1Min);
        Serial.print(F(" and "));
        Serial.print(_Start2Hour);
        Serial.print(F(":"));
        Serial.println(_Start2Min);
        Start1Hour = _Start1Hour;
        Start2Hour = _Start2Hour;
        Start1Min = _Start1Min;
        Start2Min = _Start2Min;
        if ((now.hour() >= Start1Hour) && (now.minute() >= Start1Min)) {
          Start1 = true;
        } else {
          Start1 = false;
        }
        if ((now.hour() >= Start2Hour) && (now.minute() >= Start2Min)) {
          Start2 = true;
        } else {
          Start2 = false;
        }
      } else {
        Serial.println(F("Received incorrect watering start format"));
      }
    } else if (message.sensor == DURATION_ID) {
      strncpy(WaterTime, message.data, MAX_MESSAGE_LENGTH);
      char* token = strtok(WaterTime, ":");
      _WaterDuration = atoi(token);
      token = strtok(NULL, "");
      _SoilHumTarget = atoi(token);
      if ((_WaterDuration >=0) && (_WaterDuration <20) && (_SoilHumTarget >=0) && (_SoilHumTarget <100)) {
        Serial.print(F("Received Watering duration "));
        Serial.print(_WaterDuration);
        Serial.print(F(" and max Soil Humidity "));
        Serial.println(_SoilHumTarget);
        WaterDuration = _WaterDuration;
        SoilHumTarget = _SoilHumTarget;
      } else {
        Serial.println(F("Received incorrect watering duration format"));
      }
    } else if (message.sensor == MANUAL_ID) {
      if (message.getBool() == true) {
        Manual_request = true;
        Serial.println(F("Received manual water start"));
      } else {
        water_unset = now;
        Serial.println(F("Received manual water stop"));
      }
    }
    Serial.print(F("Incoming change for sensor:"));
    Serial.print(message.sensor);
    Serial.print(F(" RSSI "));
    Serial.println(_radio.RSSI);
  }
}

void receiveTime(unsigned long time) {
  if (HAS_RTC) {
    rtc.adjust(time);
    Serial.println(F("Time Received, RTC updated"));
  } else {
    now=time;
    Serial.println(F("Time Received, internal clock updated"));
  }
  timeReceived = true;
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
    wait(98);
    heartbeat();
  }
  #else
    heartbeat();
    if (ms > MY_SMART_SLEEP_WAIT_DURATION_MS) {
      Serial.print(F("Starting sleep "));
      Serial.println(ms);
      ms-=MY_SMART_SLEEP_WAIT_DURATION_MS;
      sleep(ms,true);
      //wait(ms);
      Serial.println(F("Ending sleep"));
      //wait(ms);
      heartbeat();
    } else {
      wait (ms);
      heartbeat();
    }
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

