/***************************************************
  This is a library for the ESP_Multi_Board

  Written by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
 ****************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "ESP_Multi_Board.h"

#include <Wire.h>
#include <pgmspace.h>

#include "Adafruit_MCP23008.h"
#include "I2Cdev.h"
#include "MPU6050.h"


extern "C" {
    #include "user_interface.h"
}

Adafruit_MCP23008 mcp;
//MPU6050 accelgyro;


volatile long count_enc_r;
volatile long count_enc_l;
//Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
void changeEnc1(){

    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc1_val = 0;

    uint8_t v = (digitalRead(ENC_1_A)<<1) | digitalRead(ENC_1_B);

    enc1_val = enc1_val << 2;
    enc1_val = enc1_val | (v & 0b11);

    count_enc_r = count_enc_r - lookup_table[enc1_val & 0b1111];
}

//Encoder read idea: http://makeatronics.blogspot.com.es/2013/02/efficiently-reading-quadrature-with.html
void changeEnc2(){

    static int8_t lookup_table[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t enc2_val = 0;

    uint8_t v = (digitalRead(ENC_2_A)<<1) | digitalRead(ENC_2_B);

    enc2_val = enc2_val << 2;
    enc2_val = enc2_val | (v & 0b11);

    count_enc_l = count_enc_l + lookup_table[enc2_val & 0b1111];
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
        mcp.pullUp(p,d);
}

uint8_t ESP_Multi_Board::digitalRead(uint8_t p) {
 return mcp.digitalRead(p);
}


uint8_t ESP_Multi_Board::analogRead(uint8_t p){
    if(p>11)return 0;

  Wire.beginTransmission(MAX1605_ADDRESS); //init with slave adress
  uint8_t conf = 0 ;
  conf ^= p<<1; //toggle pin number in bits 4º, 3º, 2º, 1º
  conf |= 0x61; //set scan1=1, scan0=1 (simple read mode). set bit single-ended to 1
  //Serial.print("(");printBits(conf);Serial.print(")");
  Wire.write((byte)conf);
  Wire.endTransmission();

  Wire.requestFrom(MAX1605_ADDRESS, 1);
  return Wire.read();
}

uint8_t ESP_Multi_Board::analogScand(uint8_t size, uint8_t *table){
    if(size>11)return 0;

  Wire.beginTransmission(MAX1605_ADDRESS); //init with slave adress
  uint8_t conf = 0 ;
  conf ^= size<<1; //toggle pin number in bits 4º, 3º, 2º, 1º
  conf |= 0x01; //set scan1=0, scan0=0 (scand mode). set bit single-ended to 1
  //Serial.print("(");printBits(conf);Serial.print(")");
  Wire.write((byte)conf);
  Wire.endTransmission();

  Wire.requestFrom(MAX1605_ADDRESS, size);
  for(uint8_t i=0;i<size;i++){
    table[i]=Wire.read();
  }
  return size;
}


void ESP_Multi_Board::setMotorLeftSpeed(int velL){
    if(velL < 0){
        mcp.digitalWrite(5,HIGH);
        mcp.digitalWrite(4,LOW);
        velL=-velL;
    }else{
        mcp.digitalWrite(5,LOW);
        mcp.digitalWrite(4,HIGH);
    }
    if(velL > 1024) velL=1024;
    /*if(velL <= 20){
        velL = map(velL,0,20, 0, 50);
    }else{
        velL = map(velL,21,1023, 50, 1023);
    }*/
    analogWrite(5, velL); //set motor A high speed
}

void ESP_Multi_Board::setMotorRightSpeed(int velR){
    if(velR < 0){
        mcp.digitalWrite(7,HIGH);
        mcp.digitalWrite(6,LOW);
        velR=-velR;
    }else{
        mcp.digitalWrite(7,LOW);
        mcp.digitalWrite(6,HIGH);
    }
    if(velR > 1024) velR=1024;

    /*if(velR <= 20){
        velR = map(velR,0,20, 0, 50);
    }else{
        velR = map(velR,21,1023, 50, 1023);
    }*/
    analogWrite(2, velR); //set motor A high speed
}

void ESP_Multi_Board::setSpeed(int velL, int velR){
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

void ESP_Multi_Board::resetEncoders(){
    count_enc_r=0;
    count_enc_l=0;
}




const int MPU = 0x68; // I2C address of the MPU-6050

#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A

int16_t AcX, AcY, GyZ;
int16_t AcX_raw, AcY_raw, GyZ_raw;
int16_t AcXoffset, AcYoffset, GyZoffset;
float GyZ_integral, rotation;
unsigned long IMU_last_ts = 0;

void IMUwriteReg(byte reg, byte val) {
    Wire.beginTransmission(MPU);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

void init_IMU() {
    for (int i = 0; i < 3; i++) { // Reset the IMU a few times
        IMUwriteReg(MPU6050_PWR_MGMT_1, bit(7));   // DEVICE_RESET to 1 (D7=1)
        delay(100);
    }
    IMUwriteReg(MPU6050_PWR_MGMT_1, bit(0) | bit(1));  // set clock source to Z Gyro (D0=D1=1, D2=0) and set SLEEP to zero (D6=0, wakes up the MPU-6050)
    IMUwriteReg(MPU6050_ACCEL_CONFIG, bit(3) | bit(4));  // set sensitivity to +-16G (D3=1, D4=1) and disable high pass filter (D0,D1,D2=0)
    IMUwriteReg(MPU6050_GYRO_CONFIG, bit(3) | bit(4));  // set sensitivity to +-2000deg/s (D3=1, D4=1)
    IMUwriteReg(MPU6050_SMPLRT_DIV, 0);  // set sampling rate to 1khz (1khz / (1 + 0) = 1000 Hz)
    IMUwriteReg(MPU6050_CONFIG, bit(0) | bit(5));  // disable digital low pass filter (D0=D1=D2=0) and EXT_SYNC to GYRO_ZOUT (D3=D4=0, D5=1)
}

int16_t ESP_Multi_Board::read_IMU_GyZ(){
    int16_t new_GyZ;
    Wire.beginTransmission(MPU);
    Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true); // request a total of 2 registers
    GyZ_raw = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    GyZ = GyZ_raw - GyZoffset;
    return GyZ;
}

bool ESP_Multi_Board::read_IMU(int16_t * AcXp, int16_t * AcYp, int16_t * GyZp) {
    int16_t new_AcX, new_AcY, new_GyZ;
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3D (ACCEL_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true); // request a total of 14 registers

    // Receive and decode the response
    new_AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    new_AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    for (int i = 0; i < 8; i++) Wire.read(); // Discard 0x3F-0x46
    new_GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    bool updated = (AcX_raw != new_AcX) || (AcY_raw != new_AcY) || (GyZ_raw != new_GyZ); // Check if value has changed

    AcX_raw = new_AcX;
    AcY_raw = new_AcY;
    GyZ_raw = new_GyZ;
    AcX = AcX_raw - AcXoffset;
    AcY = AcY_raw - AcYoffset;
    GyZ = GyZ_raw - GyZoffset;
    if(AcXp) *AcXp = AcX;
    if(AcYp) *AcYp = AcY;
    if(GyZp) *GyZp = GyZ;
    return updated;
}

void ESP_Multi_Board::calibrate_IMU() {
    // Measure IMU sensor offsets (the robot must remain still)
    AcXoffset = 0;
    AcYoffset = 0;
    GyZoffset = 0;
    for (int i = 0; i < 10; i++) {
        while (read_IMU(NULL,NULL,NULL) == false) delay(10);
        AcXoffset += AcX_raw;
        AcYoffset += AcY_raw;
        GyZoffset += GyZ_raw;
    }
    AcXoffset /= 10;
    AcYoffset /= 10;
    GyZoffset /= 10;
    GyZ_integral = 0;
}

void ESP_Multi_Board::integrate_IMU() {
    while (!read_IMU(NULL,NULL,NULL));
    unsigned long ts = micros();
    float dt = ((float)(ts - IMU_last_ts)) / 1000000.; // seconds
    if (dt > 0) {
        if (dt < 0.5) GyZ_integral += GyZ * dt;
        IMU_last_ts = ts;
        rotation = GyZ_integral * 2000. / 32768.; // +-2000 deg/s sensitivity; +-2^15 full scale
    }
}

void offset_IMU(float deg) {
    GyZ_integral += deg * 32768. / 2000.;
}

void zero_IMU() {
    GyZ_integral = 0;
}


void ESP_Multi_Board::begin(void) {
    mcp.begin(4);      // use addres 4, A2=1, A1=0, A0=0
    for(int i=0;i<4;i++){
        mcp.pinMode(i, OUTPUT);
        //mcp.pullUp(i, HIGH);  // turn on a 100K pullup internally
        mcp.digitalWrite(i,LOW);
    }
    for(int i=4;i<8;i++){ //set Output Mode in the motors direccion pins
        mcp.pinMode(i, OUTPUT);
    }

    Wire.begin(4,0); //communication with MAX1605
    Wire.setClock(400000L);
    //accelgyro.initialize();
    init_IMU();
    delay(500);

    // Measure IMU sensor offsets (robot must remain still)
    calibrate_IMU();
    Serial.println("Offset:");
    Serial.println(AcYoffset);
    Serial.println(GyZoffset);

    count_enc_r=0;
    count_enc_l=0;
    pinMode(ENC_1_B,INPUT);
    pinMode(ENC_1_A,INPUT);
    pinMode(ENC_2_B,INPUT);
    pinMode(ENC_2_A,INPUT);
    attachInterrupt(ENC_1_B, changeEnc1, CHANGE);
    attachInterrupt(ENC_2_B, changeEnc2, CHANGE);

}

/*void ESP_Multi_Board::getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    accelgyro.getMotion6(ax, ay, az, gx, gy, gz);
}
int16_t ESP_Multi_Board::getRotationY(){
    return accelgyro.getRotationY();
}*/
