#include <Wire.h>
#include "Adafruit_MCP23008.h"


Adafruit_MCP23008 mcp; //modified lib to use I2C pinout of ESP8266
  
void setup() {  
  Serial.begin(115200);
  mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
  for(int i=0;i<4;i++){ //set Output Mode in the motors direccion pins
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }
  for(int i=4;i<8;i++){ //set Output Mode in the motors direccion pins
    mcp.pinMode(i, OUTPUT);
  }
  
}



void loop() {
  //set motor direccion pins
  mcp.digitalWrite(4,HIGH);
  mcp.digitalWrite(5,LOW);
  mcp.digitalWrite(6,HIGH);
  mcp.digitalWrite(7,LOW);
  analogWrite(2, 1000); //set motor A high speed
  analogWrite(5, 500);  //set motor B medium speed
  delay(3000);

  //set reverse direccion in both motors
  mcp.digitalWrite(4,LOW);
  mcp.digitalWrite(5,HIGH);
  mcp.digitalWrite(6,LOW);
  mcp.digitalWrite(7,HIGH);
  analogWrite(2, 500); //set motor A medium speed
  analogWrite(5, 1000); //set motor B high speed
  delay(3000);
  
}
