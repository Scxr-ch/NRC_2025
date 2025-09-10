#include <smorphi.h>
int front_sensor_status;
int left_sensor_status;
int right_sensor_status;
int back_sensor_status;
Smorphi my_robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  my_robot.BeginSmorphi();
}

void sensor_initialisation() {
  if ((my_robot.sm_getShape()) == ('i')) {
    front_sensor_status = my_robot.module1_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(4);
    right_sensor_status = my_robot.module2_sensor_status(0);
    back_sensor_status = my_robot.module4_sensor_status(0);
  }
}
int lastmovement = 0; // Start with Forward

void movementpattern()
{
  if(front_sensor_status == LOW && left_sensor_status == LOW)
  {
    my_robot.MoveRight(80);
    lastmovement = 3; // Right
    delay(3000);
  }
  else if(front_sensor_status == LOW && right_sensor_status == LOW)
  {
    my_robot.MoveLeft(80);
    lastmovement = 2; // Left
    delay(3000);
  }
  else if(front_sensor_status == LOW)
  {
    my_robot.MoveBackward(80);
    lastmovement = 1; // Backward
    delay(3000);
  }
  else if(back_sensor_status == LOW)
  {
    my_robot.MoveForward(80);
    lastmovement = 0; // Forward
    delay(3000);
  }
  else
  {
    if(lastmovement == 0){
      my_robot.MoveForward(80);
    }
    else if(lastmovement == 1){
      my_robot.MoveBackward(80);
    }
    else if(lastmovement == 2){
      my_robot.MoveLeft(80);
    }
    else if(lastmovement == 3){
      my_robot.MoveRight(80);
    }
  }
}
bool run = true;
void loop() {
  sensor_initialisation();
  movementpattern();
}



void hardcode(){
  while(run){
    TwoTileBack();
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
  delay(1100);
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

void TwoTileBack(){
  my_robot.MoveBackward(90);
  delay(2050);
}



