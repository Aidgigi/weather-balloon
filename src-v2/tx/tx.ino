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
//#include <TimeLib.h>

// pin defs
static const int GPS_RX_Pin = 13, GPS_TX_Pin = 12;
static const int Radio_RX_Pin = 16, Radio_TX_Pin = 17;

// bauds
static const uint32_t GPS_Baud = 9600;
static const uint32_t Radio_Baud = 57600;

// serial ports
HardwareSerial Radio_Serial(2);
HardwareSerial GPS_Serial(1);

// sensor interfaces
TinyGPSPlus gps;      // gps
Adafruit_BME280 bme;  // weather
Adafruit_MPU6050 mpu; // imu


void setup()
{
    Serial.begin(9600);
    GPS_Serial.begin(GPS_Baud, SERIAL_8N1, GPS_RX_Pin, GPS_TX_Pin);
    Radio_Serial.begin(Radio_Baud, SERIAL_8N1, Radio_RX_Pin, Radio_TX_Pin);

    // setting up sensors
    bme.begin(0x76);
    mpu.begin();

    // configuring the imu
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    Radio_Serial.write("Booted\n");
    Serial.write("Booted\n");
}

void loop()
{
    Radio_Serial.write("I am still working\n");
    Serial.write("I am still working\n");

    if (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());

        if (gps.location.isValid()) {
            Radio_Serial.write("Have GPS lock\n");
            Serial.write("Have GPS lock\n");
        }
        else {
            Radio_Serial.write("Awaiting GPS lock\n");
            Serial.write("Awaiting GPS lock\n");
        }
    }
    else {
        Radio_Serial.write("Error with GPS module\n");
        Serial.write("Error with GPS module\n");
    }

    delay(1000);
}