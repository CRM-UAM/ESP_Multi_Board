#include <ESP_Multi_Board.h>
extern "C" {
#include "user_interface.h"
}

ESP_Multi_Board robot;
os_timer_t timerSpeedController;


long leftEncoderOld = 0;
long rightEncoderOld = 0;
int velD=0;
int velI=0;

void printSpeed(void *a){
  Serial.print(velI);Serial.print(" ");
  Serial.print(velD);Serial.print(" ");
  Serial.print(robot.getEncLeftCount() - leftEncoderOld);
  leftEncoderOld=robot.getEncLeftCount();
  Serial.print(" ");
  Serial.println(robot.getEncRightCount() - rightEncoderOld);
  rightEncoderOld=robot.getEncRightCount();
}



void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
  robot.begin();
  os_timer_setfn(&timerSpeedController, printSpeed, NULL);
  os_timer_arm(&timerSpeedController, 50, true);
}



void loop() {
  // put your main code here, to run repeatedly:
  
  for(int i=0;i<1024;i+=5){
    velI=i;
    robot.setMotorLeftSpeed(velI);
    //robot.setMotorRightSpeed(rightBaseSpeed);
    delay(100);
    
  }
  velI=0;
  robot.setMotorLeftSpeed(0);
  delay(1000);

  for(int i=0;i<1024;i+=5){
    velD=i;
    robot.setMotorRightSpeed(velD);
    //robot.setMotorRightSpeed(rightBaseSpeed);
    delay(100);
    
  }

  velD=0;
  robot.setMotorRightSpeed(0);
  delay(1000);
}
