#include <ESP_Multi_Board.h>

ESP_Multi_Board robot;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial.begin(115200);
  robot.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
    long t1=micros();
    gz = robot.read_IMU_GyZ();
    long t=micros()-t1;

    //Serial.print(t); Serial.print("\t");
    //Serial.print(ay); Serial.print("\t");
    Serial.println(gz);
    
    delay(25);
}
