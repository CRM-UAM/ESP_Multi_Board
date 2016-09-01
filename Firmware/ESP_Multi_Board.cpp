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


//Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
void ESP_Multi_Board::changeEnc1(){

    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc1_val = 0;

    uint8_t v = (digitalRead(ENC_1_A)<<1) | digitalRead(ENC_1_B);

    enc1_val = enc1_val << 2;
    enc1_val = enc1_val | (v & 0b11)

    count_enc_r = count_enc_r + lookup_table[enc1_val & 0b1111];
}

//Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
void ESP_Multi_Board::changeEnc2(){

    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc2_val = 0;

    uint8_t v = (digitalRead(ENC_2_A)<<1) | digitalRead(ENC_2_B);

    enc2_val = enc2_val << 2;
    enc2_val = enc2_val | (v & 0b11)

    count_enc_l = count_enc_l + lookup_table[enc2_val & 0b1111];
}

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
    pinMode(ENC_1_B,INPUT);
    pinMode(ENC_1_A,INPUT);
    pinMode(ENC_2_B,INPUT);
    pinMode(ENC_2_A,INPUT);
    attachInterrupt(ENC_1_B, changEnc1, CHANGE);
    attachInterrupt(ENC_2_B, changEnc2, CHANGE);
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

uint8_t ESP_Multi_Board::getMotorLeftCurrent(){
    return ESP_Multi_Board::analogRead(11);
}

uint8_t ESP_Multi_Board::getMotorRightCurrent(){
    return ESP_Multi_Board::analogRead(10);
}

long ESP_Multi_Board::getEncRightCount(){
    return count_enc_r;
}

long ESP_Multi_Board::getEncLeftCount(){
    return count_enc_l;
}

void ESP_Multi_Board::resetEncRight(){
    count_enc_r=0;
}

void ESP_Multi_Board::resetEncLeft(){
    count_enc_l=0;
}

void resetEncoders(){
    count_enc_r=0;
    count_enc_l=0;
}
