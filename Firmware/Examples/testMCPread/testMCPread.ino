#include <Wire.h>
#include "Adafruit_MCP23008.h"

Adafruit_MCP23008 mcp; //modified lib to use I2C pinout of ESP8266
  
void setup() {  
  Serial.begin(115200);
  mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
  for(int i=0;i<8;i++){
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }

  
}



void loop() {
  // Print reads of all pins
   for(int i=0;i<8;i++){
    Serial.print(mcp.digitalRead(i));
    Serial.print(" , ");
  }
  Serial.println(".");
  
}
