#include <ESP_Multi_Board.h>
#include <PID_v1.h>
extern "C" {
#include "user_interface.h"
}

#define KP_MOTOR 10
#define KI_MOTOR 0
#define KD_MOTOR 4

ESP_Multi_Board robot;

boolean enLazoMotores=false;
double actualCountEncDer=0, actualCountEncIzq=0;
double objCountEncDer=0, objCountEncIzq=0;
double outIzq=0,outDer=0;
PID PIDLeft(&actualCountEncIzq, &outIzq, &objCountEncIzq, KP_MOTOR, KI_MOTOR, KD_MOTOR, DIRECT);
PID PIDRight(&actualCountEncDer, &outDer, &objCountEncDer, KP_MOTOR, KI_MOTOR, KD_MOTOR, DIRECT);
os_timer_t Timer;
void computeLazoCerradoMotores(void *ptr){
    if(!enLazoMotores)return;
    actualCountEncDer = robot.getEncRightCount();
    actualCountEncIzq = robot.getEncLeftCount();
    PIDLeft.Compute();
    PIDRight.Compute();
    robot.setMotorLeftSpeed(outIzq);
    robot.setMotorRightSpeed(outDer);
}
void initLazoCerradoMotores(){
  
   PIDLeft.SetOutputLimits(-300, 300);
   PIDRight.SetOutputLimits(-300, 300);
   PIDLeft.SetSampleTime(10);
   PIDRight.SetSampleTime(10);
   PIDLeft.SetMode(AUTOMATIC);
   PIDRight.SetMode(AUTOMATIC);
   os_timer_setfn(&Timer, computeLazoCerradoMotores, NULL);
   os_timer_arm(&Timer, 20, true);
}

void moveDistance(long rightEncTicks, long leftEncTicks){
    objCountEncDer=rightEncTicks;
    objCountEncIzq=leftEncTicks;
    enLazoMotores=true;
}
void setup() {
  // put your setup code here, to run once:
  delay(500);
  
  Serial.begin(115200);
  robot.begin();
  initLazoCerradoMotores();
}

void loop() {
  // put your main code here, to run repeatedly:
 moveDistance(robot.getEncRightCount()+250,+robot.getEncLeftCount()+250);
 delay(15000);
 //moveDistance(250,250);
 //delay(5000);
}
