/***************************************************
  This is a library for the ESP_Multi_Board

  Written by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
 ****************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <pgmspace.h>

#include "Adafruit_MCP23008.h"
#include "ESP_Multi_Board.h"



void ESP_Multi_Board::begin(void) {
    mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
    for(int i=0;i<4;i++){ //set Output Mode in the motors direccion pins
        mcp.pinMode(i, INPUT);
        mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
    }
    for(int i=4;i<8;i++){ //set Output Mode in the motors direccion pins
        mcp.pinMode(i, OUTPUT);
    }

    Wire.begin(4,0); //communication with MAX1605
    count_enc_r=0;
    count_enc_l=0;

}

void ESP_Multi_Board::pinMode(uint8_t p, uint8_t d) {
    if(p<4)
        mcp.pinMode(p,d);
}

uint8_t ESP_Multi_Board::readGPIO(void) {
  return mcp.readGPIO();
}

void ESP_Multi_Board::writeGPIO(uint8_t gpio) {
  mcp.writeGPIO(gpio);
}


void ESP_Multi_Board::digitalWrite(uint8_t p, uint8_t d) {
  if(p<4)
    mcp.digitalWrite(p,d);
}

void ESP_Multi_Board::pullUp(uint8_t p, uint8_t d) {
    if(p<4)
        mpc.pullUp(p,d);
}

uint8_t ESP_Multi_Board::digitalRead(uint8_t p) {
 return mcp.digitalRead(p);
}


uint8_t ESP_Multi_Board::analogRead(uint8_t p){
    if(p>11)return 0;

  Wire.beginTransmission(MAX1605_ADDRESS); //init with slave adress
  uint8_t conf = 0 ;
  conf ^= p<<1; //toggle pin number in bits 4º, 3º, 2º, 1º
  conf |= 0x61; //set scan1=1, scan0=1. set bit single-ended to 1
  //Serial.print("(");printBits(conf);Serial.print(")");
  Wire.write((byte)conf);
  Wire.endTransmission();

  Wire.requestFrom(MAX1605_ADDRESS, 1);
  return Wire.read();
}

void ESP_Multi_Board::setMotorLeftSpeed(int velL){
    if(velL < 0){
        mcp.digitalWrite(4,HIGH);
        mcp.digitalWrite(5,LOW);
        velL=-velL;
    }else{
        mcp.digitalWrite(4,LOW);
        mcp.digitalWrite(5,HIGH);
    }
    if(velL > 1024) velL=1024;
    analogWrite(5, velL); //set motor A high speed
}

void ESP_Multi_Board::setMotorRightSpeed(int velR){
    if(velR < 0){
        mcp.digitalWrite(6,HIGH);
        mcp.digitalWrite(7,LOW);
        velR=-velR;
    }else{
        mcp.digitalWrite(6,LOW);
        mcp.digitalWrite(7,HIGH);
    }
    if(velR > 1024) velR=1024;
    analogWrite(2, velR); //set motor A high speed
}

void ESP_Multi_Board::setSpeed(int velR, int velL){
    setMotorLeftSpeed(velL);
    setMotorRightSpeed(velR);
}

uint8_t getMotorLeftCurrent(){
    return analogRead(11);
}

uint8_t getMotorRightCurrent(){
    return analogRead(10);
}
