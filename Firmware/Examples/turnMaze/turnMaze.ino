#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <pgmspace.h>
#include <ESP8266WiFi.h>
#include <ESP_Multi_Board.h>

extern "C" {
#include "user_interface.h"
}

//const char* ssid = "ONOFBA9";
//const char* password = "4125905522uceda";
const char* ssid = "wifiVictor";
const char* password = "wifivictor";



WiFiServer server(23);
WiFiClient serverClient;

ESP_Multi_Board robot;
os_timer_t timerSpeedController;


#define SIZE_TELEMETRIA 120//40*30 30seg telemetry
#define DIM_TELEMETRIA 8
float telemetria[SIZE_TELEMETRIA+1][DIM_TELEMETRIA]={0};
int p_telemetria=0;

//call speedProfile(void) in systick handle to make it execute every one milli second
//this controller sample code doesn't include

//here are the code you need to call periodically to make the mouse sample and excute and a constant rate
//in order to make sample and pwm updating at a constant rate, you'd better call the code in the handler of a timer periodically
//the period for intterupt handler is usually 1ms, which means you have to finish all code excuting within 1ms
//I simply call the controller code in systick handler that intterupts every 1ms, notice systick timer is also used to
//keep track the system time in millisecond
/*
//test the timing make sure everything finishes within 1ms and leave at least 30% extra time for other code to excute in main routing
void systickHandler(void)
{
    Millis++;  //keep track of the system time in millisecond
    if(bUseIRSensor)
        readIRSensor();
    if(bUseGyro)
        readGyro();
    if(bUseSpeedProfile)
        speedProfile
}
*/

//ps. if you want to change speed, you simply change the targetSpeedX and targetSpeedW in main routing(int main) at any time.



long distanceLeft=0;
float curSpeedX = 0;
float curSpeedW = 0;
int targetSpeedX = 0;
int targetSpeedW = 0;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputX = 0;
float pidInputW = 0;
float posErrorX = 0;
float posErrorW = 0;
float posErrorWir = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
float oldPosErrorWir = 0;
int posPwmX = 0;
int posPwmW = 0;
int posPwmWir = 0;

#define TARGET_WALL_DIST 70
float IR_WEIGHT = 0.0;
const float ir_weight_straight = 1.0;
const float ENCODER_GYRO_WEIGHT = 1.0;

float kpX = 0.95, kdX = 10;
float kpW = 0.8, kdW = 17;//used in straight
const float kpW0 = kpW, kdW0 = kdW;//used in straight
float kpWir = 0.6, kdWir = 6.5;//used with IR errors
const float kpW1 = 0.85, kdW1 = 16;//used for T1 and T3 in curve turn
const float kpW2 = 0.8, kdW2 = 20;//used for T2 in curve turn

float accX = 45;//6m/s/s
float decX = 35;
float accW = 1; //cm/s^2
float decW = 1;

long encoderCount=0;
long encoderChange=0;
long oldEncoderCount=0;
long leftEncoderChange=0;
long rightEncoderChange=0;
long leftEncoderOld = 0;
long rightEncoderOld = 0;
long leftEncoder = 0;
long  rightEncoder = 0;
long leftEncoderCount = 0;
long rightEncoderCount = 0;
    
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;

#define R_WHEEL 15.65 //radius in mm
#define ONE_CELL_DISTANCE 27500/(2*3.14*R_WHEEL)
#define speed_to_counts(a) (120*(a)/(2*3.14*R_WHEEL))
#define counts_to_speed(a) ((a)*(2*3.14*R_WHEEL)/120)
//#define abs(x)  (x<0)?(-x):(x)

//speed to count or count to speed are the macro or function to make the unit conversion
// between encoder_counts/ms and mm/ms or any practical units you use.
int moveSpeed = speed_to_counts(0);
int turnSpeed = speed_to_counts(12);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(15);


float gyroFeedbackRatio = -223.5;//5900;

void getEncoderStatus()
{
    long leftEncoder = robot.getEncLeftCount(); //leftEncoder = TIM2->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
    long rightEncoder = robot.getEncRightCount(); //rightEncoder = TIM5->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5



    leftEncoderChange = leftEncoder - leftEncoderOld;
    rightEncoderChange = rightEncoder - rightEncoderOld;
    encoderChange = (leftEncoderChange + rightEncoderChange)/2;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;
    encoderCount =  (leftEncoderCount+rightEncoderCount)/2;

    distanceLeft -= encoderChange;// update distanceLeft
}



void updateCurrentSpeed(void)
{
    if(curSpeedX < targetSpeedX && curSpeedX >= 0){
        curSpeedX += (float)(speed_to_counts(accX*2)/100);
        if(curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX > targetSpeedX && curSpeedX > 0){
        curSpeedX -= (float)speed_to_counts(decX*2)/100;
        if(curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX < targetSpeedX && curSpeedX < 0){
        curSpeedX += (float)speed_to_counts(decX*2)/100;
        if(curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }else if(curSpeedX > targetSpeedX && targetSpeedX <= 0){
        curSpeedX -= (float)(speed_to_counts(accX*2)/100);
        if(curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }
    
    if(curSpeedW < targetSpeedW && curSpeedW >= 0){
        curSpeedW += accW;
        if(curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW > targetSpeedW && curSpeedW > 0){
        curSpeedW -= decW;
        if(curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW < targetSpeedW && curSpeedW < 0){
        curSpeedW += decW;
        if(curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }else if(curSpeedW > targetSpeedW && curSpeedW <= 0){
        curSpeedW -= accW;
        if(curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }
    
}

void leerIRs(uint8_t *data){
  uint8_t IR_value1[3]={0};
  uint8_t IR_value2[3]={0};
  robot.analogScand(3, IR_value1);  
  robot.digitalWrite(0,HIGH);
  robot.analogScand(3, IR_value2);
  robot.digitalWrite(0,LOW);
  for(int i=0;i<3;i++){
    data[i] = (IR_value2[i] - IR_value1[i]);
  }
}
inline void getErrIRNew(){
  int DLMiddleValue=101;
  int DRMiddleValue=170;
    int DLMinValue=60;
    int DRMinValue=76;
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    telemetria[p_telemetria][6]=IR_value[0];
    telemetria[p_telemetria][7]=IR_value[2];

    if( abs(DLMiddleValue - IR_value[0]) < abs(IR_value[2] - DRMiddleValue) ){ //pared izqueirda mas cercana que la derecha, la utilizo para ir por el centro
      if(IR_value[0] > DLMinValue) //pared detectada por encima del umbral minimo 
        posErrorWir = DLMiddleValue - IR_value[0];
      else //pared no detectada
        posErrorWir /= 3; 
    }else{
      if(IR_value[2] > DRMinValue )
        posErrorWir = IR_value[2] - DRMiddleValue;
      else
        posErrorWir /= 3;
    }
}

inline void getErrIRChino(){
      int DLMiddleValue=101;
    int DRMiddleValue=170;
    int DLMinValue=60;
    int DRMinValue=76;
    
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    
    telemetria[p_telemetria][6]=IR_value[0];
    telemetria[p_telemetria][7]=IR_value[2];

    if(IR_value[0] > DLMinValue || IR_value[2] > DRMinValue){
      if(IR_value[0] > IR_value[2]){
        posErrorWir = DLMiddleValue - IR_value[0];
      }else{
        posErrorWir = IR_value[2] - DRMiddleValue;
      }
      
    }else{
      posErrorWir=0;
    }
}

inline void getErrIR(){
    int DLMiddleValue=101;
    int DRMiddleValue=170;
    int DLMinValue=60;
    int DRMinValue=76;
    
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    
    telemetria[p_telemetria][6]=IR_value[0];
    telemetria[p_telemetria][7]=IR_value[2];
    
    if(IR_value[0] > DLMinValue && IR_value[2] < DRMinValue){
      posErrorWir = DLMiddleValue - IR_value[0];
      if(IR_value[0] < DLMiddleValue)
        posErrorWir /=3;
    }else if(IR_value[2] > DRMinValue && IR_value[0] < DLMinValue){
      posErrorWir = IR_value[2] - DRMiddleValue;
      if(IR_value[2] < DRMiddleValue)
        posErrorWir /=3;
    }else if(IR_value[0] > DLMinValue && IR_value[2] > DRMinValue){
      posErrorWir = 0;
      
      if(IR_value[0] < DLMiddleValue)
        posErrorWir += (DLMiddleValue - IR_value[0])/3;
      else 
        posErrorWir += (DLMiddleValue - IR_value[0]);
      
      if(IR_value[2] < DRMiddleValue)
        posErrorWir += (IR_value[2] - DRMiddleValue)/3;
      else
        posErrorWir += (IR_value[2] - DRMiddleValue);
        
      posErrorWir /= 2;
      
    }else
      posErrorWir /= 3;
}

void calculateMotorPwm(void) // encoder PD controller
{
    int sensorFeedback;
    
    //Serial.println(IR_value[0]);
    /* simple PD loop to generate base speed for both motors */
    encoderFeedbackX = rightEncoderChange + leftEncoderChange;
    encoderFeedbackW = rightEncoderChange - leftEncoderChange;
    float gyro = robot.read_IMU_GyZ()*1.0;
    float gyroFeedback = gyro / gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
    //gyroIntegrate += gyroFeedback;
    //sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system

    //if(onlyUseGyroFeedback)
    //    rotationalFeedback = gyroFeedback;
    //else if(onlyUseEncoderFeedback)
          float rotationalFeedback = /*((float)(ENCODER_GYRO_WEIGHT*1.0f)*(encoderFeedbackW*1.0f)) + ((float)(1.0f - ENCODER_GYRO_WEIGHT*1.0)*(*/gyroFeedback;//));
    //else
    //    rotationalFeedback = encoderFeedbackW + gyroFeedback;
        //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
        //make sure to check the sign of sensor error.
  
    posErrorX += curSpeedX - encoderFeedbackX;
    posErrorW += curSpeedW - rotationalFeedback;
   


    getErrIRChino();
    /*if(posErrorWir < 20){
      posErrorW = 0;
    }*/
     



    
    posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
    posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
    posPwmWir = kpWir * posErrorWir + kdWir * (posErrorWir - oldPosErrorWir); 

    oldPosErrorX = posErrorX;
    oldPosErrorW = posErrorW;
    oldPosErrorWir = posErrorWir;

    leftBaseSpeed = posPwmX - ( (1-IR_WEIGHT)*posPwmW + IR_WEIGHT*posPwmWir);
    rightBaseSpeed = posPwmX + ( (1-IR_WEIGHT)*posPwmW + IR_WEIGHT*posPwmWir);

    robot.setMotorLeftSpeed(leftBaseSpeed);
    robot.setMotorRightSpeed(rightBaseSpeed);
    if(p_telemetria<SIZE_TELEMETRIA){
      telemetria[p_telemetria][0]=micros();
      telemetria[p_telemetria][1]=curSpeedX;
      telemetria[p_telemetria][2]=encoderFeedbackX;
      //telemetria[p_telemetria][3]=curSpeedW;
      telemetria[p_telemetria][3]=curSpeedW;
      telemetria[p_telemetria][4]=rotationalFeedback;
      telemetria[p_telemetria][5]=posErrorWir;
      //telemetria[p_telemetria][6]=leftEncoderChange;
      p_telemetria++;
    }
    
}


float needToDecelerate(long dist, int curSpd, int endSpd)//speed are in encoder counts/ms, dist is in encoder counts
{
    if (curSpd<0) curSpd = -curSpd;
    if (endSpd<0) endSpd = -endSpd;
    if (dist<0) dist = 1;//-dist;
    if (dist == 0) dist = 1;  //prevent divide by 0

    return (abs(counts_to_speed((curSpd*curSpd - endSpd*endSpd)*100/(double)dist/4/2))); //dist_counts_to_mm(dist)/2);
    //calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
    //use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
    //because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

void resetSpeedProfile(void)
{
    //resetEverything;
    //disable sensor data collecting functions running in 1ms interrupt
//    useSensor = false;
//    useGyro = false;
    //no PID calculating, no motor lock
//    usePID = false;

    robot.setSpeed(0,0);//setLeftPwm(0);//setRightPwm(0);

    pidInputX = 0;
    pidInputW = 0;
    curSpeedX = 0;
    curSpeedW = 0;
    targetSpeedX = 0;
    targetSpeedW = 0;
    posErrorX = 0;
    posErrorW = 0;
    oldPosErrorX = 0;
    oldPosErrorW = 0;
    leftEncoderOld = 0;
    rightEncoderOld = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderCount = 0;
    oldEncoderCount = 0;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;

    robot.resetEncoders();//TIM2->CNT = 0;//reset left encoder count //TIM5->CNT = 0;//reset right encoder count
}

void speedProfile(void *a){ 
    getEncoderStatus(); 
    updateCurrentSpeed(); 
    calculateMotorPwm(); 
}

/*void getSensorEror(void)//the very basic case
{
    if(DLSensor > DLMiddleValue && DRSensor < DRMiddleValue)
        sensorError = DLMiddleValue - DLSensor;
    else if(DRSensor > DRMiddleValue && DLSensor < DLMiddleValue)
        sensorError = DRSensor - DRMiddleValue;
    else
        sensorError = 0;
}*/


/*
sample code for straight movement
*/
void moveOneCell()
{
    targetSpeedW = 0;
    targetSpeedX = moveSpeed;
    kpW = kpW0;
    kdW = kdW0;
    IR_WEIGHT = ir_weight_straight; // usar los IR para alinearte con la pared
    do
    {
        /*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)
        here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
        /*sample*/
        float accMin = needToDecelerate(ONE_CELL_DISTANCE-(encoderCount-oldEncoderCount), curSpeedX, moveSpeed);
        
        if( accMin < decX)
            targetSpeedX = maxSpeed;
        else
            targetSpeedX = moveSpeed;
            
        Serial.print(millis());Serial.print(" ");
        Serial.print(ONE_CELL_DISTANCE-(encoderCount-oldEncoderCount));Serial.print(" ");
        Serial.print(curSpeedX);Serial.print(" ");
        Serial.print(moveSpeed);Serial.print(" ");
        Serial.print(accMin);Serial.print(" ");
        Serial.print(decX);Serial.print(" ");
        Serial.print(targetSpeedX);Serial.println(" ");
        delay(5);
        //there is something else you can add here. Such as detecting falling edge of post to correct longitudinal position of mouse when running in a straight path
        //Serial.print(targetSpeedX);Serial.print(" ");Serial.print((encoderCount-oldEncoderCount));Serial.print(" ");Serial.print(ONE_CELL_DISTANCE);Serial.println("");
     }
    while( ( (encoderCount-oldEncoderCount) < ONE_CELL_DISTANCE )//&& LFSensor < LFvalue2 && RFSensor < RFvalue2)//use encoder to finish 180mm movement if no front walls
    //|| (LFSensor < LFvalues1 && LFSensor > LFvalue2)//if has front wall, make the mouse finish this 180mm with sensor threshold only
    //|| (RFSensor < RFvalue1 && RFSensor > RFvalve2)
    );
    //LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
    //LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
    //and LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting
    //these thresholds just in case the readings are too weak.

    Serial.print("Final Encoder Count");Serial.println((encoderCount-oldEncoderCount));
    oldEncoderCount = encoderCount; //update here for next movement to minimized the counts loss between cells.
}

void turn90(int dir){
  float velW=7.4*dir;
  long t1 = (speed_to_counts(abs(velW))/accW)*25;
  long t2 = 510;
  long t3 = (speed_to_counts(abs(velW))/decW)*25;
  long tini = millis();
  IR_WEIGHT = 0; //no usar los IR para alinearte con la pared cuando estas girando
  targetSpeedX = turnSpeed;
  targetSpeedW = speed_to_counts(velW);
  kpW = kpW1;
  kdW = kdW1;
  while(millis()-tini < t1){delay(1);}
  tini = millis();
  kpW = kpW2;
  kdW = kdW2;
  while(millis()-tini < t2){delay(1);}
  tini = millis();
  kpW = kpW1;
  kdW = kdW1;
  targetSpeedW = 0;
  targetSpeedX = targetSpeedX/4;
  while(millis()-tini < t3){delay(1);}
  targetSpeedX = 0;
  kpW = kpW0;
  kdW = kdW0;
  oldEncoderCount = encoderCount;
}

void turn180(int dir){
  float velW=16*dir;
  long t1 = (speed_to_counts(abs(velW))/accW)*25;
  long t2 =310;
  long t3 = (speed_to_counts(abs(velW))/decW)*25;
  long tini = millis();
  IR_WEIGHT = 0; //no usar los IR para alinearte con la pared cuando estas girando
  targetSpeedX = 0;
  targetSpeedW = speed_to_counts(velW);
  kpW = kpW1;
  kdW = kdW1;
  while(millis()-tini < t1){delay(1);}
  tini = millis();
  kpW = kpW2;
  kdW = kdW2;
  while(millis()-tini < t2){delay(1);}
  tini = millis();
  kpW = kpW1;
  kdW = kdW1;
  targetSpeedW = 0;
  targetSpeedX = 0;
  while(millis()-tini < t3){delay(1);}
  targetSpeedX = 0;
  kpW = kpW0;
  kdW = kdW0;
  oldEncoderCount = encoderCount;
}


void init_telnet(){
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  server.begin();
  server.setNoDelay(true);
  Serial.print("\nReady! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
}

void print_tl(String str){
  if (serverClient && serverClient.connected()){
        serverClient.print(str);
   }
}

void check_send_telnet_telemetry(){
  if (server.hasClient()){
    if (!serverClient || !serverClient.connected()){
        if(serverClient) serverClient.stop();
        serverClient = server.available();
        Serial.println("New client: ");
      }
  }

  //check clients for data
  /*if (serverClient && serverClient.connected()){
    if(serverClient.available()){
      //get data from the telnet client
      while(serverClient.available()){
          serverClient.read();
      }
    }
  }*/

//Send data
  if (serverClient && serverClient.connected()){
    for(int i=0;i<p_telemetria;i++){
      for(int j=0;j<DIM_TELEMETRIA;j++){
        serverClient.print(telemetria[i][j]);
        serverClient.print(" ");
      }
      serverClient.println("");
    }
    serverClient.println("");
    p_telemetria=0;
    delay(5);
   }
  
}

void leerParedes(){
    uint8_t IR_value[3]={0};
    leerIRs(IR_value);
    
    
}

void setup() {
  // put your setup code here, to run once:
  delay(500);

  Serial.begin(115200);
  init_telnet();
  
  robot.begin();

   os_timer_setfn(&timerSpeedController, speedProfile, NULL);
   os_timer_arm(&timerSpeedController, 25, true);
   resetSpeedProfile();
}

#define TIME_BT_MOVE 10
int num_loop=1;
int dir=1;
void loop() {
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  moveOneCell();
  delay(TIME_BT_MOVE);
  /*turn90(1);
  delay(TIME_BT_MOVE);
  turn90(1);
  delay(TIME_BT_MOVE);*/
 
  /*if((num_loop++)%3==0){
    targetSpeedX=0;
    delay(10000);
  }*/
  check_send_telnet_telemetry();
  //delay(2000);

  
  
  /*for(int i=0;i<p_telemetria;i++){
    for(int j=0;j<5;j++){
      Serial.print(telemetria[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }*/
  //p_telemetria=0;
  
  
}
