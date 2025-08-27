#include <smorphi.h>

Smorphi my_robot;

void setup(){
  Serial.begin(115200);
  my_robot.BeginSmorphi();
}

void loop(){
  int front_sensor_status = my_robot.module0_sensor_status(1);
  Serial.println(front_sensor_status);
  if (front_sensor_status == HIGH) {
  }

}