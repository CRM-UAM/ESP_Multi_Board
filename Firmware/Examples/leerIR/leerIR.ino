#include <ESP_Multi_Board.h>


ESP_Multi_Board robot;

void setup() {
  // put your setup code here, to run once:
  delay(500);
  
  Serial.begin(115200);
  robot.begin();

}
uint8_t medida2=0;
uint8_t medida1=0;
uint8_t IR_value1[3]={0};
uint8_t IR_value2[3]={0};
void loop() {
  long t1= micros();
  //medida1 = robot.analogRead(0); //
  robot.analogScand(3, IR_value1);  
  robot.digitalWrite(0,HIGH);
  //delayMicroseconds(40);
  //medida2 =robot.analogRead(0); //
  robot.analogScand(3, IR_value2);
  robot.digitalWrite(0,LOW);
  long t2= micros();
  
  //Serial.print(t2-t1);Serial.print(" , ");Serial.print(medida1);Serial.print(" ");Serial.print(medida2);Serial.print(" ");
  //Serial.println(medida2-medida1); 
  Serial.print("MEDIDAS: ");
  for(int i=0;i<3;i++){
    Serial.print(IR_value2[i] - IR_value1[i]);
    Serial.print(" ");
  }
  Serial.println("");

  delay(100);

}
