#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <Adafruit_MPU6050.h>
#include <AsyncTimer.h>
#include <Adafruit_INA260.h>
#include <MPU6050_light.h>
#include <string.h>

//#include <TimeLib.h>

// pin defs
static const int GPS_RX_Pin = 13, GPS_TX_Pin = 12;
static const int Radio_RX_Pin = 16, Radio_TX_Pin = 17;

static const int UV_SNS = 4;

// bauds
static const uint32_t GPS_Baud = 9600;
static const uint32_t Radio_Baud = 57600;

// serial ports
HardwareSerial Radio(2);
HardwareSerial GPS_Serial(1);

// sensor interfaces
TinyGPSPlus gps;      // gps
Adafruit_BME280 bme;  // weather
MPU6050 mpu(Wire); // imu
Adafruit_INA260 power_main = Adafruit_INA260();
Adafruit_INA260 power_tx = Adafruit_INA260();

// file interfaces
File statusFile;
File attitudeFile;
File positionFile;
File weatherFile;
File powerFile;

void setup()
{
    Serial.begin(9600);
    GPS_Serial.begin(GPS_Baud, SERIAL_8N1, GPS_RX_Pin, GPS_TX_Pin);
    Radio.begin(Radio_Baud, SERIAL_8N1, Radio_RX_Pin, Radio_TX_Pin);
    bool sd_status = SD.begin();

    bool gps_status = GPS_Serial.available() > 0;
    bool radio_status = Radio.available() > 0;

    // setting up sensors
    //bme.begin(0x76);
    Wire.begin();
    bool mpu_status = mpu.begin();

    bool power_main_status = power_main.begin((uint8_t)64);
    bool power_tx_status = power_tx.begin((uint8_t)65);

    bool bme_status = bme.begin(0x76);

    mpu.calcOffsets();

    // init log files
    InitSD();

    statusFile.close();
    attitudeFile.close();
    positionFile.close();
    weatherFile.close();
    powerFile.close();

    // status report 
    WriteBoth("Initialization status:\n");
    // gps
    PrintBoth("GPS - ");
    if (gps_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // radio
    PrintBoth("Radio - ");
    if (radio_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // sd
    PrintBoth("SD - ");
    if (sd_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // mpu
    PrintBoth("MPU - ");
    if (mpu_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // bme
    PrintBoth("BME - ");
    if (bme_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // power_main
    PrintBoth("Power Main - ");
    if (power_main_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // power_tx
    PrintBoth("Power TX - ");
    if (power_tx_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }
}

unsigned long previousMillis = 0;
// for sensors
static unsigned long gpsInterval = 1000;
// for logging
static unsigned long attitudeInterval = 50;
static unsigned long positionInterval = 1000; // since gps can only update at 1 Hz by default
static unsigned long weatherInterval  = 100;
static unsigned long powerInterval = 500;
static unsigned long radioStatusInterval = 10000; // every 10 seconds we'll check rssi/noise
// for sending
static unsigned long sendInterval = 2000;

void loop() {
  // reading in/parsing GPS data
  while (GPS_Serial.available() > 0)
  {
    gps.encode(GPS_Serial.read());    
  }

  mpu.update();

  Radio.print("Latitude: ");
  Radio.println(String(gps.location.lat(), 6));
  Radio.print("Longtitude: ");
  Radio.println(String(gps.location.lng(), 6));
  Radio.print("Altitude: ");
  Radio.println(String(gps.altitude.meters(), 1));
  Radio.println("Satellite count: ");
  Radio.println(String(gps.satellites.value()));

  delay(5000);
}

void InitSD()
{
  unsigned long start = millis();

  char *paths[6] = { "/status.csv", "/attitude.csv", "/position.csv", "/weather.csv", "/power.csv" };
  File files[6] = { statusFile, attitudeFile, positionFile, weatherFile, powerFile };

  int folderIndex = 1;
  String statusPath = "/" + String(folderIndex) + "/status.csv";

  char buffer[50];
  statusPath.toCharArray(buffer, 50);

  while (SD.exists(buffer))
  {
    folderIndex++;
    
    statusPath = "/" + String(folderIndex) + "/status.csv";
    statusPath.toCharArray(buffer, 50);
  }
  
  String rootPath = "/" + String(folderIndex);
  char rootPathBuffer[50];
  rootPath.toCharArray(rootPathBuffer, 50);
  SD.mkdir(rootPathBuffer);
  
  for (int i=0; i<5; i++)
  {
    char absPath[100];
    strcpy(absPath, rootPathBuffer);
    strcat(absPath, paths[i]);

    File file = fs::FS(SD).open(absPath, FILE_WRITE);
    file.print("");
    file.close();
    
    files[i] = fs::FS(SD).open(absPath, FILE_APPEND);
  }

  //Serial.println(statusFile.path());

  statusFile = files[0];
  attitudeFile = files[1];
  positionFile = files[2];
  weatherFile = files[3];
  powerFile = files[4];

  statusFile.println("Type,Message");
  attitudeFile.println("Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AngleX,AngleY,AngleZ,InternalTemp");
  positionFile.println("Time,Lat,Lng,Alt,Speed,Course,Sats");
  weatherFile.println("Time,Temp,Pres,Hum,Alt");
  powerFile.println("Time,V,I,P");

  for (int i=0; i<5; i++)
  {
    files[i].flush();
  }

}

// this will write to tel radio and usb serial port
void WriteBoth(const char * message) {
  Serial.write(message);
  Radio.write(message);
}

void PrintBoth(const char * message) {
  Serial.print(message);
  Radio.print(message);
}