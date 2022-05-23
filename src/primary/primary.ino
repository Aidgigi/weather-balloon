// imports
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <AsyncTimer.h>
#include <TimeLib.h>

#define TINY_GSM_MODEM_SIM800
#include <TinyGSM.h>

#define E220_30
#define FREQUENCY_915
#define POWER_30

#include "LoRa_E22.h"

long time_offset  = -25200;

// serial pin defs
static const int GPSRXPin = 13, GPSTXPin = 12;
static const int LoRaRXPin = 12, LoRaTXPin = 13;

// I2C pin defs
static const int SDAPin = 21, SCLPin = 22;

// baud rate defs
static const uint32_t GPSBaudRate = 9600;
static const uint32_t LoRaBaudRate = 9600;
//static const uint32_t SDBaudRate = 9600;

// other defs
static const uint32_t LoRaFreq = 915E6;

// vars
long lat, lng, elv;
double temp, pressure, hum, alt;
double gyrox, gyroy, gyroz, accx, accy, accz;
bool gpsReady;
int satcount;

// object defs
TinyGPSPlus gps;
Adafruit_BME280 bme;  // temp/pressure/hum sensor
Adafruit_MPU6050 mpu; // imu

AsyncTimer t, z, tSync;

LoRa_E22 lora(&Serial2);

//serial ports
HardwareSerial GPSSerial(1);
//HardwareSerial LRSerial(1);
//HardwareSerial SDSerial(2);



// time stuff
byte last_second, Second, Minute, Hour, Day, Month;
int Year;

bool timeSetInt = false;





// im terrible at this programming language so this is how the state is gonna be managed
// it'll be a fucking int, and you aren't going to complain. thank you.
// states
// -1 - error starting up.
// 0 - starting up, waiting for GPS/data acquire. handshake exchanged every 20 sec.
// 1 - flight ready, ready message sent at this point (mobile data). telemetry sent every 30 sec.
// 2 - climbing, determined by increasing altitude[s]. telemetry sent evert  5 sec.
// 3 - apex, will be hard to implement due to altitude measurements BS. telemetry every 5 sec.
// 4 - falling, caused by decreasing elevation. telemetry as often as possible.
// 5 - pre-land, below 1000 ft. begin try data reacquire. telemetry as often and as loud as possible.
// 6 - landed, no more velocity. telemetry every 30 sec. as loud as possible. data updates every 2 min.

int state = -1; // start the state at -1, if setup fails and state is still -1, we know something fucked up

// Ideas:
// loop() will simply read the state and act accordingly, with each loop ending by doing some checks
// the checks will update the state by reading whatever we need to determine the state
// for example, if we go from climbing to falling...  we know that we've passed the apex and are
// now returning to earth. in this case, state goes from 3 to 4.

// "loop() will simply read the state and act accordingly" read state -> perform actions according to state
// really, all we need to do is transmit every so often, maybe change power, and try to use mobile data (only at the end)


// main program
void setup() {
  Serial.begin(9600);
  GPSSerial.begin(GPSBaudRate, SERIAL_8N1, GPSRXPin, GPSTXPin);
  lora.begin();
  //LRSerial.begin(LoRaBaudRate, SERIal_8N1, LoRaRXPin, LoRaTXPin);
  bme.begin(0x76);
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // setting up SD card
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }

  if(SD.exists("/gps_data.txt") == 0)
  {
    writeFile(SD, "/gps_data.txt", "Begin\n");
  }
  if(SD.exists("/mpu_data.txt") == 0)
  {
    writeFile(SD, "/mpu_data.txt", "Begin\n");
  }
  if(SD.exists("/bme_data.txt") == 0)
  {
    writeFile(SD, "/bme_data.txt", "Begin\n");
  }

  state = 0;
  t.setInterval(LogTelemetry, 50);
  z.setInterval(SendTelemetry, 10000);
  tSync.setInterval(SyncTime, 300000);
}

void LogTelemetry()
{
  if(gpsReady)
  {
    String timeOut = (String)month() + "/" + (String)day() + "/" + (String)year() + "  " + (String)hour() + ":" + (String)minute() + ":" + (String)second();
    String gpsOut = timeOut + "  --  Latitude " + (String)lat + ", Longitude: " + (String)lng + ", Elevation: " + (String)elv + ", Links: " + (String)satcount + "\n";
    Serial.println(timeOut);
    appendFile(SD, "/gps_data.txt", gpsOut.c_str());
  }

  String mpuOut = "GyroX: " + (String)gyrox + ", GyroY: " + (String)gyroy + ", GyroZ: " + (String)gyroz + "\nAccX: " + (String)accx + ", AccY: " + (String)accy + ", AccZ: " + (String)accz + "\n";
  String bmeOut = "Temp: " + (String)temp + ", Pressure: " + (String)pressure + ", Humidity: " + (String)hum + ", (Calculated) Altitude: " + (String)alt + "\n";

  appendFile(SD, "/bme_data.txt", bmeOut.c_str());
  appendFile(SD, "/mpu_data.txt", mpuOut.c_str());
}


void test()
{
  Serial.println("Hello!");
}

void loop() {
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
    {
      UpdateGPS();
    }

  
  if (!gpsReady)
  {
    Serial.println("Waiting for GPS lock...");
    delay(500);
    return;
  }

  if (!timeSetInt)
  {
    setTime(Hour, Minute, Second, Day, Month, Year);
    adjustTime(time_offset);
    timeSetInt = true;
  }
    
  UpdateBME();
  UpdateMPU();

  t.handle();
  z.handle();
  tSync.handle();
}



// communication methods
void SendTelemetry()
{
  String timo = (String)hour() + ":" + (String)minute() + ":" + (String)second();
  String gpso = "G " + (String)lat + " " + (String)lng + " " + (String)elv;
  String tmpo = "T " + (String)temp + " " + (String)pressure + " " + (String)hum + " " + (String)alt;
  String mpuo = "M " + (String)gyrox + " " + (String)gyroy + " " + (String)gyroz;

  String n = "\n";
  String comb = n + timo + n + gpso + n + tmpo + n + mpuo;
  ResponseStatus rs = lora.sendMessage(comb);
  Serial.println(rs.getResponseDescription());
}

void SyncTime()
{
  setTime(Hour, Minute, Second, Day, Month, Year);
  adjustTime(time_offset);
}

// update sensor stuff, this is stored in some global vars
void UpdateGPS()
{
  gpsReady = gps.location.isValid() && gps.time.isValid() && gps.date.isValid();
  if (!gpsReady)
  {
    return;
  }

  lat = gps.location.lat();
  lng = gps.location.lng();
  elv = gps.altitude.meters();
  satcount = gps.satellites.value();


  Second = gps.time.second();
  Minute = gps.time.minute();
  Hour = gps.time.hour();

  Day = gps.date.day();
  Month = gps.date.month();
  Year = gps.date.year();
}

void UpdateBME()
{
  temp = bme.readTemperature();
  pressure = bme.readPressure() / 100.0f;
  hum = bme.readHumidity();
  alt = bme.readAltitude(1013.25);
}

void UpdateMPU()
{
  sensors_event_t a, g, tempr;
  mpu.getEvent(&a, &g, &tempr);

  gyrox = g.gyro.x;
  gyroy = g.gyro.y;
  gyroz = g.gyro.z;

  accx = a.acceleration.x;
  accy = a.acceleration.y;
  accz = a.acceleration.z;
}




// SD card utils
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
