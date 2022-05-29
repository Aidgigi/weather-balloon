#include <SPI.h>
#include <SD.h>
#include <FS.h>

#define E22_30
#define FREQUENCY_915
//#define POWER_30
#include "LoRa_E22.h"

LoRa_E22 lora(&Serial2);

bool loraReady = false;
bool testRan = false;

void setup() {
  Serial.begin(9600);
  lora.begin();

  // setting up SD card
//  if(!SD.begin(5)){
//    Serial.println("Card Mount Failed");
//    //return;
//  }

//  if(SD.exists("/telemetry.txt") == 0)
//  {
//    writeFile(SD, "/telemetry.txt", "Begin\n");
//  }
}

void loop() {
  if (!loraReady)
    LoraConfig();

  if (lora.available()>1) {
    ResponseContainer rc = lora.receiveMessageRSSI();
    if (rc.status.code!=1)
    {
        Serial.println(rc.status.getResponseDescription());
    }
    else
    {
        Serial.println(rc.data);
        Serial.println(rc.rssi, DEC);
        //appendFile(SD, "/telemetry.txt", rc.data.c_str());
    }
  }
//  if (Serial.available()) {
//    Serial2.write(Serial.read());
//  }
}

void test()
{
  ResponseStatus rs = lora.sendMessage("This is a test");
  Serial.println(rs.getResponseDescription());
  Serial.println("Transmitted");
}

void LoraConfig()
{
  // configing the module
  ResponseStructContainer c = lora.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;

  configuration.ADDL = 0x03;
  configuration.ADDH = 0x00;

  configuration.CHAN = 23;

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_240_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  //configuration.OPTION.transmissionPower = POWER_30;
  
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  
  lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE); // set config
  lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);

  // test config
  c = lora.getConfiguration();
  configuration = *(Configuration*) c.data;
  Serial.println(configuration.OPTION.transmissionPower);       

  loraReady = true;
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
