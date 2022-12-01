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
#include <TimeLib.h>

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

// define QNH here
const float qnh = 1020;

// char arrays
char lat_array[16];
char lng_array[16];
char alt_array[16];
char spd_array[16];
char vrt_array[16];
char crs_array[16];

char acc_x_array[16];
char acc_y_array[16];
char acc_z_array[16];
char gyro_x_array[16];
char gyro_y_array[16];
char gyro_z_array[16];
char angle_x_array[16];
char angle_y_array[16];
char angle_z_array[16];
char it_array[16];

char ot_array[16];
char p_array[16];
char hu_array[16];
char b_alt_array[16];

char main_voltage_array[16];
char main_current_array[16];
char main_power_array[16];
char tx_voltage_array[16];
char tx_current_array[16];
char tx_power_array[16];


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

    // init gps
    InitGPS();

    // status report 
    WriteBoth("Initialization status:\n");
    // gps
    PrintBoth("GPS        - ");
    if (gps_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // radio
    PrintBoth("Radio      - ");
    if (radio_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // sd
    PrintBoth("SD         - ");
    if (sd_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // mpu
    PrintBoth("MPU        - ");
    if (mpu_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }

    // bme
    PrintBoth("BME        - ");
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
    PrintBoth("Power TX   - ");
    if (power_tx_status) {
      PrintBoth("Good\n");
    } else {
      PrintBoth("Bad\n");
    }
}

unsigned long att_prev_millis = 0;
unsigned long pos_prev_millis = 0;
unsigned long wea_prev_millis = 0;
unsigned long pow_prev_millis = 0;
unsigned long tel_prev_millis = 0;
unsigned long fls_prev_millis = 0;
// for sensors
static unsigned long gpsInterval = 1000;
// for logging
static unsigned long attitude_interval = 100;
static unsigned long position_interval = 1000; // since gps can only update at 1 Hz by default
static unsigned long weather_interval  = 100;
static unsigned long power_interval = 500;
static unsigned long radio_status_interval = 10000; // every 10 seconds we'll check rssi/noise
// for sending
static unsigned long send_interval = 5000;
// for saving
static unsigned flush_interval = 1000;

// storing sensor values
float lat, lng, gpsAlt, course, speed;
unsigned long t_last_altitude;
float vertical_speed; // for calculating vertical velocity
int sats;

float accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ, intTemp;

float extTemp, pressure, humidity, baroAlt;

float v_main, i_main, p_main, v_tx, i_tx, p_tx;

// time zone offset


void loop() {
  bool gps_updated = false;

  // reading in/parsing GPS data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
    gps_updated = true;
  }
  
  if (gps_updated) {
    UpdateGPS();
  }

  // updating time
  if (gps.location.age() < 500) {
    SetTime();
  }

  // updating gps and mpu
  mpu.update();


  // timed tasks
  unsigned long current_millis = millis();

  if (current_millis - att_prev_millis >= attitude_interval) {
    UpdateMPU();
    LogAttitude();

    att_prev_millis = millis();
  }

  if (current_millis - pos_prev_millis >= position_interval) {
    LogPosition();

    pos_prev_millis = millis();
  }

  if (current_millis - wea_prev_millis >= weather_interval) {
    UpdateBME();
    LogWeather();

    wea_prev_millis = millis();
  }

  if (current_millis - pow_prev_millis >= power_interval) {
    UpdatePower();
    LogPower();

    pow_prev_millis = millis();
  }

  if (current_millis - tel_prev_millis >= send_interval) {
    UpdateMPU();
    UpdateBME();
    UpdatePower();
    SendTelemetry();

    tel_prev_millis = millis();
  }

  if (current_millis - fls_prev_millis >= flush_interval)
  {
    FlushSD();

    fls_prev_millis = millis();
  }
}


// compiling all data and sending it
void SendTelemetry() {
  char payload[500];
  // time formatting
  char time_buffer[16];
  char timestamp_buffer[64];
  GetTime(time_buffer);
  GetTimeStamp(timestamp_buffer);
  sprintf(payload, "1:{%s}|2:{%s}", time_buffer, timestamp_buffer);
  // location formatting
  char loc_buffer[128];
  sprintf(loc_buffer, "|3:{%s}|4:{%s}|5:{%s}|6:{%s}|7:{%s}|8:{%s}|9:S{%i}", lat_array, lng_array, alt_array, spd_array, vrt_array, crs_array, sats);
  strcat(payload, loc_buffer);
  // orientation
  char angle_buffer[128];
  sprintf(angle_buffer, "|10:{%s}|11:{%s}|12:{%s}|13:{%s}", angle_x_array, angle_y_array, angle_z_array, it_array);
  strcat(payload, angle_buffer);
  // weather
  char weather_buffer[128];
  sprintf(weather_buffer, "|14:{%s}|15:{%s}|16:{%s}|17:{%s}", ot_array, p_array, hu_array, b_alt_array);
  strcat(payload, weather_buffer);
  // power
  char power_buffer[128];
  sprintf(power_buffer, "|18:{%s}|19:{%s}|20:{%s}|21:{%s}|22:{%s}|23:{%s}", main_voltage_array, main_current_array, main_power_array, tx_voltage_array, tx_current_array, tx_power_array);
  strcat(payload, power_buffer);

  Radio.println(payload);
}


// logs attitude
void LogAttitude() {
  char log_buffer[512];
  char time_buffer[16];

  TelToChars();
  GetTime(time_buffer);

  sprintf(log_buffer, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", time_buffer, acc_x_array, acc_y_array, acc_z_array, gyro_x_array, gyro_y_array, gyro_z_array, angle_x_array, angle_y_array, angle_z_array, it_array);

  attitudeFile.println(log_buffer);
}

void LogPosition() {
  char log_buffer[256];
  char time_buffer[16];

  TelToChars();
  GetTime(time_buffer);

  sprintf(log_buffer, "%s,%s,%s,%s,%s,%s,%s,%i", time_buffer, lat_array, lng_array, alt_array, spd_array, vrt_array, crs_array, sats);

  positionFile.println(log_buffer);
}

void LogWeather() {
  char log_buffer[256];
  char time_buffer[16];

  TelToChars();
  GetTime(time_buffer);

  sprintf(log_buffer, "%s,%s,%s,%s,%s", time_buffer, ot_array, p_array, hu_array, b_alt_array);

  weatherFile.println(log_buffer);
}

void LogPower() {
  char log1_buffer[256];
  char log2_buffer[256];
  char time_buffer[16];

  TelToChars();
  GetTime(time_buffer);

  sprintf(log1_buffer, "%s,Main,%s,%s,%s", time_buffer, main_voltage_array, main_current_array, main_power_array);
  sprintf(log2_buffer, "%s,TX,%s,%s,%s", time_buffer, tx_voltage_array, tx_current_array, tx_power_array);

  powerFile.println(log1_buffer);
  powerFile.println(log2_buffer);
}


// converts all current telemetry to chars
void TelToChars() {
  dtostrf(lat, 0, 6, lat_array);
  dtostrf(lng, 0, 6, lng_array);
  dtostrf(gpsAlt, 0, 2, alt_array);
  dtostrf(speed, 0, 2, spd_array);
  dtostrf(vertical_speed, 0, 2, vrt_array);
  dtostrf(course, 0, 1, crs_array);

  dtostrf(accX, 0, 4, acc_x_array);
  dtostrf(accY, 0, 4, acc_y_array);
  dtostrf(accZ, 0, 4, acc_z_array);
  dtostrf(gyroX, 0, 4, gyro_x_array);
  dtostrf(gyroY, 0, 4, gyro_y_array);
  dtostrf(gyroZ, 0, 4, gyro_z_array);
  dtostrf(angleX, 0, 2, angle_x_array);
  dtostrf(angleY, 0, 2, angle_y_array);
  dtostrf(angleZ, 0, 2, angle_z_array);
  dtostrf(intTemp, 0, 2, it_array);

  dtostrf(extTemp, 0, 2, ot_array);
  dtostrf(pressure, 0, 3, p_array);
  dtostrf(humidity, 0, 3, hu_array);
  dtostrf(baroAlt, 0, 2, b_alt_array);

  dtostrf(v_main, 1, 2, main_voltage_array);
  dtostrf(i_main, 1, 2, main_current_array);
  dtostrf(p_main, 1, 2, main_power_array);
  dtostrf(v_tx, 1, 2, tx_voltage_array);
  dtostrf(i_tx, 1, 2, tx_current_array);
  dtostrf(p_tx, 1, 2, tx_power_array);
}

void GetTime(char* outBuffer) {
  memset(outBuffer, 0, sizeof(outBuffer));
  sprintf(outBuffer, "%02d:%02d:%02d",  hour(), minute(), second());
}

void GetTimeStamp(char* outBuffer) {
  memset(outBuffer, 0, sizeof(outBuffer));
  sprintf(outBuffer, "%lu", gps.time.value());
}

// sets the time using gps
void SetTime() {
  int Year = gps.date.year();
  int Month = gps.date.month();
  int Day = gps.date.day();

  int Hour;
  int HourPre = gps.time.hour();

  if (HourPre < 8) {
    Hour = 24 - (8 - HourPre);
  } else {
    Hour = HourPre - 8;
  }

  int Minute = gps.time.minute();
  int Second = gps.time.second();
  
  setTime(Hour, Minute, Second, Day, Month, Year);
}


// gets the location and calculates vertical velocity
// also gets horizontal velocity and course
void UpdateGPS() {
  // location
  lat = gps.location.lat();
  lng = gps.location.lng();
  float new_alt = gps.altitude.meters();

  // other
  course = gps.course.deg();
  speed = gps.speed.mps();
  sats = gps.satellites.value();

  // calculate vertical speed
  unsigned long current_millis = millis();
  unsigned long delta_time = current_millis - t_last_altitude;
  float displacement = new_alt - gpsAlt;

  if (displacement < 0)
  {
    displacement = -1.0 * displacement;
  }

  if (delta_time > 0UL && displacement > 0)
  {
    vertical_speed = displacement / float(delta_time / 1000.0);
  }

  gpsAlt = new_alt;
  t_last_altitude = current_millis;
}


// gets MPU values
void UpdateMPU() {
  // acceleration
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  // gyro acc
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();

  // angle
  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();

  // temp
  intTemp = mpu.getTemp();
}


// gets power sensors values
void UpdatePower() {
  // read in main
  v_main = power_main.readBusVoltage();
  i_main = power_main.readCurrent();
  p_main = power_main.readPower();

  // read in tx
  v_tx = power_tx.readBusVoltage();
  i_tx = power_tx.readCurrent();
  p_tx = power_tx.readPower();
}

// gets BME values
void UpdateBME() {
  extTemp = bme.readTemperature();
  pressure = bme.readPressure();
  humidity = bme.readHumidity();

  baroAlt = bme.readAltitude(qnh);
}


// saves all data on buffer to SD
void FlushSD() {
  File files[5] = { statusFile, attitudeFile, positionFile, weatherFile, powerFile };

  for (int i=0; i<5; i++)
  {
    files[i].flush();
  }
}


// starts the SD card
void InitSD()
{
  char *paths[5] = { "/status.csv", "/attitude.csv", "/position.csv", "/weather.csv", "/power.csv" };
  File files[5] = { statusFile, attitudeFile, positionFile, weatherFile, powerFile };

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

  statusFile.println("Type,Message,Value");
  attitudeFile.println("Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AngleX,AngleY,AngleZ,InternalTemp");
  positionFile.println("Time,Lat,Lng,Alt,Speed,VSpeed,Course,Sats");
  weatherFile.println("Time,Temp,Pres,Hum,Alt");
  powerFile.println("Time,Battery,V,I,P");

  for (int i=0; i<5; i++)
  {
    files[i].flush();
  }
}




// ----------------------------------------------
// WEIRD GPS SHIT -- DO NOT ENTER




// config checksum tool
void ChecksumUBLOX(uint8_t *data) // Assumes buffer is large enough and modifyable
{
  int i, length;
  uint8_t a, b;

  a = 0; // Clear initial checksum bytes
  b = 0;

  length = data[4] + (data[5] << 8); // 16-bit Payload Length
  
  for(i=2; i<(length + 6); i++) // Sum over body
  {
    a += data[i];
    b += a;
  }

  data[i+0] = a; // Write checksum bytes into tail
  data[i+1] = b;
}

// need to set a specific mode of the GPS so that the altitude is unlocked
void InitGPS() {
  // payload
  uint8_t ubx_cfg_nav5[] = { // CFG-NAV5 (06 24)
  0xB5,0x62,0x06,0x24,0x24,0x00, // Header/Command/Size
  0x01,0x00,0x06,0x00,0x00,0x00,0x00,0x00, // Payload data (Dynamics flag, and Dynamic setting)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x55,0xB4 }; // Checksum (Fletcher) of preceeding packet

  GPS_Serial.flush();
  ChecksumUBLOX(ubx_cfg_nav5);
  GPS_Serial.write(ubx_cfg_nav5, sizeof(ubx_cfg_nav5));
  
  delay(500);

  uint8_t verify_command[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
  GPS_Serial.write(verify_command, sizeof(verify_command));

  delay(500);

  #ifdef GPS_LOGGIN
  Serial.println("\n");
  while (true) {
    while (GPS_Serial.available() > 0)
    {
      for (int i = 0; i<7; i++) {
        char a = GPS_Serial.read();
        if (a != '\n'){
          Serial.print((char)a);
          Serial.print(" ");
        }        
      }

      Serial.print("      ");

      bool has = false;

      for (int i = 0; i<7; i++) {
        byte a = GPS_Serial.read();
        if (a == 0xb5) {
          has = true;
        }
        if ((char)a != '\n');
        {
          Serial.printf("%02x ", a);
        }
      }

      if (has) {
        Serial.print("     *");
      }
      
      Serial.println();
    }
  }
  #endif
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


// hiding this shit away