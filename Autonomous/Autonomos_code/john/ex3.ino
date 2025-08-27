#include <smorphi.h>

Smorphi my_robot;

void setup(){
  Serial.begin(115200);
  my_robot.BeginSmorphi();
}

void loop(){
  int front_sensor_status = my_robot.module1_sensor_status(0);
  Serial.println(front_sensor_status);
  if (front_sensor_status == HIGH) {
    Serial.println("Nothing is in my way!");
    my_robot.MoveForward(10);
  }
  else {
    Serial.println("Fuck this shit!");
    my_robot.stopSmorphi();
  }

}