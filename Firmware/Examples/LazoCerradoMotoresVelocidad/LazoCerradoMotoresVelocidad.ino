#include <PID_v1.h>
#include <ESP_Multi_Board.h>


ESP_Multi_Board robot;

double VelObjetivo, VelEnc, Output;
long lastValEnc=0, lastTime=0;
int vel=0;
double Kp=0.01, Ki=0, Kd=0.005;
PID myPID(&VelEnc, &Output, &VelObjetivo, Kp, Ki, Kd, DIRECT);


void setLeftSpeed(double pps){
  long actualValEnc = robot.getEncLeftCount();
  long delayTime = (millis() - lastTime);
  VelEnc = (actualValEnc - lastValEnc)*1000/delayTime;
  Serial.print(actualValEnc - lastValEnc);Serial.print(", ");
  VelObjetivo = pps;
  
  lastValEnc = actualValEnc;
  lastTime=millis(); 
  
  myPID.Compute();
  vel+=Output;
  robot.setMotorLeftSpeed((int)vel);
  
   Serial.print(delayTime);Serial.print(" ");Serial.print(VelEnc);Serial.print(" ");Serial.print(VelObjetivo);Serial.print(" "); Serial.print(Output);Serial.print(" "); Serial.println(vel);
}

void setup() {
  delay(500);
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.begin();
  myPID.SetOutputLimits(-1023, 1023);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  //robot.setSpeed(0,0);
  //delay(100);
  //Serial.print(robot.getEncRightCount());Serial.print(" ");Serial.print(robot.getEncLeftCount());Serial.print(" ");Serial.println(robot.getEncRightCount() - robot.getEncLeftCount());
  delay(20);
  setLeftSpeed(30*12);
}
