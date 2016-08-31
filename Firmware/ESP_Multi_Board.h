/***************************************************
  This is a library for the ESP_Multi_Board

  Written by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
 ****************************************************/

#ifndef _ESP_MULTI_BOARD_H
#define _ESP_MULTI_BOARD_H
// Don't forget the Wire library
#include <Wire.h>
#include "Adafruit_MCP23008.h"


class ESP_Multi_Board {
public:

  void begin(void);

  void pinMode(uint8_t p, uint8_t d);
  void digitalWrite(uint8_t p, uint8_t d);
  void pullUp(uint8_t p, uint8_t d);
  uint8_t digitalRead(uint8_t p);
  uint8_t readGPIO(void);
  void writeGPIO(uint8_t);

  uint8_t analogRead(uint8_t p);

  void setSpeed(int velR, int velL);
  void setMotorLeftSpeed(int velL);
  void setMotorRightSpeed(int velR);
  unsigned long getEncRightCount(); //TODO
  unsigned long getEncLeftCount(); //TODO
  uint8_t getMotorLeftCurrent();
  uint8_t getMotorRightCurrent();

  //IMU functions //TODO

 private:
  Adafruit_MCP23008 mcp;
  unsigned long count_enc_r;
  unsigned long count_enc_l;

};

#define MCP23008_ADDRESS 4
#define MAX1605_ADDRESS 0x65

#endif
