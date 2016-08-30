#include <Wire.h>
#include "Adafruit_MCP23008.h"


Adafruit_MCP23008 mcp; //modified lib to use I2C pinout of ESP8266
  
void setup() {  
  Serial.begin(115200);
  mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
  for(int i=0;i<8;i++){
    mcp.pinMode(i, OUTPUT);
  }
}



void loop() {
  //blink all digital pins
  for(int i=0;i<8;i++){
    mcp.digitalWrite(i,HIGH);
  }
  delay(1000);
  for(int i=0;i<8;i++){
    mcp.digitalWrite(i,LOW);
  }
  delay(1000);
  
}
