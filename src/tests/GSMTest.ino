#include <HardwareSerial.h>

HardwareSerial gsm(1);
String Grsp, Arsp;

void setup() {
  Serial.begin(9600);
  gsm.begin(9600, SERIAL_8N1, 16, 17);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  gsm.print("AT+CMGF=1\r");
  delay(100);
  gsm.print("AT+CMGS=\"+xxxxxxxxxxx\"\r");
  delay(100);
  gsm.print("This working?");
  delay(100);
  gsm.print((char)26);
  delay(100);
  gsm.println();
}

void loop() {
  if(gsm.available())
  {
    Grsp = gsm.readString();
    Serial.println(Grsp);
  }

  if(Serial.available())
  {
    Arsp = Serial.readString();
    gsm.println(Arsp);
  }
}
