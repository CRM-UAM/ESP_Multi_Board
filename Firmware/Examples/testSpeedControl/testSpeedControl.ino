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

const char* ssid = "ONOFBA9";
const char* password = "4125905522uceda";

WiFiServer server(23);
WiFiClient serverClient;

ESP_Multi_Board robot;
os_timer_t timerSpeedController;


#define SIZE_TELEMETRIA 180
long telemetria[SIZE_TELEMETRIA+1][4]={0};
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
float oldPosErrorX = 0;
float oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;
float kpX = 0.4, kdX = 7;
float kpW = 0.8, kdW = 12;//used in straight
float kpW1 = 1;//used for T1 and T3 in curve turn
float kdW1 = 26;
float kpW2 = 1;//used for T2 in curve turn
float kdW2 = 45;
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
#define ONE_CELL_DISTANCE (130*120)/(2*3.14*R_WHEEL)*5
#define speed_to_counts(a) (120*(a)/(2*3.14*R_WHEEL))
#define counts_to_speed(a) ((a)*(2*3.14*R_WHEEL)/120)
//#define abs(x)  (x<0)?(-x):(x)

//speed to count or count to speed are the macro or function to make the unit conversion
// between encoder_counts/ms and mm/ms or any practical units you use.
int moveSpeed = speed_to_counts(0);
int turnSpeed = speed_to_counts(50);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(18);


int gyroFeedbackRatio = 5700;//5900;

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
    if(curSpeedX < targetSpeedX)
    {
        curSpeedX += (float)(speed_to_counts(accX*2)/100);
        if(curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }
    else if(curSpeedX > targetSpeedX)
    {
        curSpeedX -= (float)speed_to_counts(decX*2)/100;
        if(curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }
    if(curSpeedW < targetSpeedW)
    {
        curSpeedW += accW;
        if(curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }
    else if(curSpeedW > targetSpeedW)
    {
        curSpeedW -= decW;
        if(curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }
}


void calculateMotorPwm(void) // encoder PD controller
{
    int gyroFeedback;
    int rotationalFeedback;
    int sensorFeedback;

    /* simple PD loop to generate base speed for both motors */
    encoderFeedbackX = rightEncoderChange + leftEncoderChange;
    encoderFeedbackW = rightEncoderChange - leftEncoderChange;

    //gyroFeedback = aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
    //sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system

    //if(onlyUseGyroFeedback)
    //    rotationalFeedback = gyroFeedback;
    //else if(onlyUseEncoderFeedback)
        rotationalFeedback = encoderFeedbackW;
    //else
    //    rotationalFeedback = encoderFeedbackW + gyroFeedback;
        //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
        //make sure to check the sign of sensor error.
  
    posErrorX += curSpeedX - encoderFeedbackX;
    posErrorW += curSpeedW - rotationalFeedback;

    posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
    posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);

    oldPosErrorX = posErrorX;
    oldPosErrorW = posErrorW;

    leftBaseSpeed = posPwmX - posPwmW;
    rightBaseSpeed = posPwmX + posPwmW;

    robot.setMotorLeftSpeed(leftBaseSpeed);
    robot.setMotorRightSpeed(rightBaseSpeed);
    if(p_telemetria<SIZE_TELEMETRIA){
      telemetria[p_telemetria][0]=micros();
      telemetria[p_telemetria][1]=curSpeedX;
      telemetria[p_telemetria][2]=encoderFeedbackX;
      //telemetria[p_telemetria][3]=curSpeedW;
      telemetria[p_telemetria][3]=rotationalFeedback;
      //telemetria[p_telemetria][5]=rightEncoderChange;
      //telemetria[p_telemetria][6]=leftEncoderChange;
      p_telemetria++;
    }
    
}


int needToDecelerate(long dist, int curSpd, int endSpd)//speed are in encoder counts/ms, dist is in encoder counts
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
    //enable_sensor(),
    //enable_gyro();
    //enable_PID();

    targetSpeedW = 0;
    targetSpeedX = moveSpeed;

    do
    {
        /*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)
        here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
        /*sample*/
        
        if(needToDecelerate(ONE_CELL_DISTANCE-(encoderCount-oldEncoderCount), curSpeedX, moveSpeed) < decX)
            targetSpeedX = maxSpeed;
        else
            targetSpeedX = moveSpeed;
      
        delay(25);
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

    oldEncoderCount = encoderCount; //update here for next movement to minimized the counts loss between cells.
}

void speedProfile(void *a)
{
    //Serial.println(micros());
    getEncoderStatus();
    //Serial.print("VEL_ENC: ");Serial.println(encoderChange);
    updateCurrentSpeed();
    //Serial.print("VEL_DESEADA: ");Serial.print(curSpeedX);Serial.print(" ");Serial.println(targetSpeedX);
    calculateMotorPwm();
    //Serial.print("VEL_ENC_X: ");Serial.println(encoderFeedbackX);
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
      for(int j=0;j<4;j++){
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

void loop() {
  moveOneCell();
  delay(2000);
  /*for(int i=0;i<p_telemetria;i++){
    for(int j=0;j<4;j++){
      Serial.print(telemetria[i][j]);
      Serial.print(" ");
    }
    Serial.println("");
  }*/
  check_send_telnet_telemetry();
 // delay(5000);
}
