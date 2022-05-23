// LoRa config
#define E220_30
#define FREQUENCY_915
#define POWER_30

#include "LoRa_E22.h"

LoRa_E22 lora(&Serial2);

void setup() {
  Serial.begin(9600);
  lora.begin();
}

void loop() {
  if (lora.available()>1) {
    ResponseContainer rc = lora.receiveMessage();
    if (rc.status.code!=1)
    {
        Serial.println(rc.status.getResponseDescription());
    }
    else
    {
        Serial.println(rc.data);
    }
  }
//  if (Serial.available()) {
//    Serial2.write(Serial.read());
//  }
}
