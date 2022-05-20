#include "FS.h"
#include "SD.h"
#include "SPI.h"

void setup() {
  Serial.begin(9600);

  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }

  writeFile(SD, "/test.txt", "This is working!!!");
}

void loop() {
  Serial.println("Working!");
  delay(2000);
}

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
