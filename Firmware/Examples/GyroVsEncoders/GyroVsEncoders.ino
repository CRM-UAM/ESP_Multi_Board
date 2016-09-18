#include <ESP_Multi_Board.h>
extern "C" {
#include "user_interface.h"
}

#include <ESP8266WiFi.h>


ESP_Multi_Board robot;
os_timer_t timerSpeedController;

#define SIZE_TELEMETRIA 500
long telemetria[SIZE_TELEMETRIA+1][3]={0};
int p_telemetria=0;



const char* ssid = "wifiVictor";
const char* password = "wifivictor";

WiFiServer server(23);
WiFiClient serverClient;

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
      for(int j=0;j<3;j++){
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

long leftEncoderOld = 0;
long rightEncoderOld = 0;

void readValues(void *a){
  
  long t1=millis();
  long leftEncoder = robot.getEncLeftCount(); //leftEncoder = TIM2->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
  long rightEncoder = robot.getEncRightCount(); //rightEncoder = TIM5->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5
  long leftEncoderChange = leftEncoder - leftEncoderOld;
  long rightEncoderChange = rightEncoder - rightEncoderOld;
  leftEncoderOld = leftEncoder;
  rightEncoderOld = rightEncoder;

  int16_t gz = robot.read_IMU_GyZ();
  
  if(p_telemetria < (SIZE_TELEMETRIA-1)){
      telemetria[p_telemetria][0]=t1;
      telemetria[p_telemetria][1]=gz;
      telemetria[p_telemetria][2]=rightEncoderChange - leftEncoderChange;
      p_telemetria++;
    }
}
void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(115200);
  robot.begin();
  init_telnet();
  delay(500);
  
  os_timer_setfn(&timerSpeedController, readValues, NULL);
  os_timer_arm(&timerSpeedController, 25, true);
  
  robot.setMotorLeftSpeed(300);
  robot.setMotorRightSpeed(-300);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(p_telemetria >= SIZE_TELEMETRIA-1){
    check_send_telnet_telemetry();
  }
}
