#include <Wire.h>
#include "Adafruit_MCP23008.h"


Adafruit_MCP23008 mcp; //modified lib to use I2C pinout of ESP8266
  
void setup() {  
  Serial.begin(115200);
  mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
  Wire.begin(4,0);
  for(int i=0;i<8;i++){ //set Output Mode in the motors direccion pins
    mcp.pinMode(i, OUTPUT);
  }

  //INIT MAX11605E+
  //Wire.beginTransmission(0x65); //init with slave adress
  
}

void printBits(byte myByte){
 for(byte mask = 0x80; mask; mask >>= 1){
   if(mask  & myByte)
       Serial.print('1');
   else
       Serial.print('0');
 }
}

uint8_t readAnalog(uint8_t p){
  //Wire.beginTransmission(0x65); //init with slave adress
  //Wire.endTransmission();
  if(p>=12)return 0;
  
  Wire.beginTransmission(0x65); //init with slave adress
  uint8_t conf = 0 ; 
  conf ^= p<<1; //toggle pin number in bits 4º, 3º, 2º, 1º
  conf |= 0x61; //set scan1=1, scan0=1. set bit single-ended to 1
  
  //Serial.print("(");printBits(conf);Serial.print(")");
  
  Wire.write((byte)conf);
  Wire.endTransmission();
  
  Wire.requestFrom(0x65, 1);
  return Wire.read();
}

void loop() {
  //set motor direccion pins
  for(int i=0;i<8;i++){ //set Output Mode in the motors direccion pins
   mcp.digitalWrite(i,HIGH);
  }
  for(int i=0;i<12;i++){
    Serial.print(readAnalog(i));
    Serial.print(" , ");
  }
  Serial.println("");
  delay(20);

  
  
}
