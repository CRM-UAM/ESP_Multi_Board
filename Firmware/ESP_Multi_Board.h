/***************************************************
  This is a library for the ESP_Multi_Board

  Written by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
 ****************************************************/

#ifndef _ESP_MULTI_BOARD_H

#define _ESP_MULTI_BOARD_H





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
  uint8_t analogScand(uint8_t size, uint8_t *table);

  void move(long rightEncTicks, long leftEncTicks);
  void setSpeed(int velR, int velL);
  void setMotorLeftSpeed(int velL);
  void setMotorRightSpeed(int velR);
  long getEncRightCount();
  long getEncLeftCount();
  void resetEncRight();
  void resetEncLeft();
  void resetEncoders();

  uint8_t getMotorLeftCurrent();
  uint8_t getMotorRightCurrent();
  //IMU functions
  //void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
  //int16_t getRotationY();
  void calibrate_IMU();
  int16_t read_IMU_GyZ();
  bool read_IMU(int16_t *AcXp, int16_t *AcYp, int16_t *GyZp);
  void integrate_IMU();

 private:


};

#define MCP23008_ADDRESS 4
#define MAX1605_ADDRESS 0x65
#define ENC_1_A 16
#define ENC_2_A 14
#define ENC_1_B 12
#define ENC_2_B 13




#endif
