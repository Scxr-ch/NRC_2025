#include <smorphi.h>
Smorphi my_robot;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  my_robot.BeginSmorphi();
}
bool run = true;
void loop() {
  // put your main code here, to run repeatedly:
  while(run){
  OneTileBack();
  my_robot.stopSmorphi();
  run = false;
  }
  
}
//moves 23.5cm
void OneTileForward(){
  my_robot.MoveForward(90);
  delay(900);
}
//48.5cm
void TwoTileForward(){
  my_robot.MoveForward(90);
  delay(1900);
}
//23.5, but slightly slanted
void OneTileLeft(){
  my_robot.MoveLeft(90);
  delay(1150);
  my_robot.CenterPivotLeft(90);
  delay(470);
}
//Moves 24cm to the right 
void OneTileRight(){
  my_robot.MoveRight(90);
  delay(1210);
  my_robot.CenterPivotRight(90);
  delay(1300);
}
//48.5cm to the left
void TwoTileLeft(){
  my_robot.MoveLeft(90);
  delay(2250);
  my_robot.CenterPivotLeft(90);
  delay(500);
}
//47cm to the right, but moves up abit 
void TwoTileRight(){
  my_robot.MoveRight(90);
  delay(2300);
  my_robot.CenterPivotRight(90);
  delay(1050);
}

void OneTileBack(){
  my_robot.MoveBackward(90);
  delay(940);
}
