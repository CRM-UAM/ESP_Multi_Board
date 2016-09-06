#include <ESP_Multi_Board.h>


ESP_Multi_Board robot;

void setup() {
  // put your setup code here, to run once:
  delay(500);
  
  Serial.begin(115200);
  robot.begin();

}
int medida2=0;
void loop() {
  int medida1 = robot.analogRead(0);
  
  robot.digitalWrite(0,HIGH);
  //unsigned long t1 = micros();
  //delayMicroseconds(40);
  medida2 =robot.analogRead(0);
  robot.digitalWrite(0,LOW);
  //unsigned long t2 = micros();
  //Serial.print(t2-t1);Serial.print(" , ");Serial.print(medida1);Serial.print(" ");Serial.print(medida2);Serial.print(" ");
  Serial.println(medida2-medida1);

  delay(100);

}
