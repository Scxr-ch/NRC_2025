#include <smorphi.h>
Smorphi my_robot;
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
HUSKYLENS huskylens;
SoftwareSerial myHuskySerial(19, 18);

void printResult(HUSKYLENSResult result){
if (result.command == COMMAND_RETURN_BLOCK){
Serial.println(result.ID);
}

else if (result.command == COMMAND_RETURN_ARROW){
Serial.println("Wrong mode");
}
else{
Serial.println("Object unknown!");
}
}
int color_signature;
int command_block;


int front_left_sensor_status;
int front_right_sensor_status;

/*
int right_sensor_status;

int rear_sensor_status;

int left_sensor_status;


String last_DIR;
*/

// Describe this function...
void movements() {
  /*
  if (left_sensor_status == 0 && right_sensor_status == 0) {
    my_robot.MoveForward(60);
  }
   else if ((left_sensor_status == 0 && right_sensor_status == 0) && front_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
   else if (front_sensor_status == 0 && right_sensor_status == 0) {
    my_robot.CenterPivotLeft(270);
    delay(1000);
  }
   else if (front_sensor_status == 0 && left_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
   else if (right_sensor_status == 0) {
    my_robot.CenterPivotLeft(270);
    delay(10);
  }
   else if (left_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(10);
  }
   else if (front_sensor_status == 0) {
    my_robot.CenterPivotRight(270);
    delay(1000);
  }
   else {
    my_robot.MoveForward(60);
  }
  */
  my_robot.MoveForward(60);
}

// Describe this function...
/*
void I_mvmts() {
  if (front_sensor_status == 0 && right_sensor_status == 0) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.MoveLeft(80);
    last_DIR = "DONE";
  }
   else if (front_sensor_status == 0 && left_sensor_status == 0) {
    my_robot.MoveRight(80);
    last_DIR = "Right";
  }
   else if (right_sensor_status == 0) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.MoveForward(60);
    last_DIR = "Fwd";
  }
   else if (left_sensor_status == 0) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.MoveForward(60);
  }
   else if (left_sensor_status == 0 && last_DIR == "DONE") {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.MoveForward(60);
  }

}


// Describe this function...
void sensor_initialisation() {
  if ((my_robot.sm_getShape()) == ('i')) {
    front_sensor_status = my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(10);
    I_mvmts();
  }

  if ((my_robot.sm_getShape()) == ('o')) {
    front_sensor_status = my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(6);
    left_sensor_status = my_robot.module4_sensor_status(0);
    movements();
  }

}

*/



void setup() {
  Serial.begin(115200);
  my_robot.BeginSmorphi();
  myHuskySerial.begin(9600);
	while (!huskylens.begin(myHuskySerial)){
	Serial.println(F("Begin failed!"));
	Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
	Serial.println(F("2.Please recheck the connection."));
	delay(100);
}

}

void loop() {

  int x = 0;
  if (huskylens.request()) {
  if (huskylens.available())  {
  HUSKYLENSResult result = huskylens.read();
  printResult(result);
  if (result.command == COMMAND_RETURN_BLOCK) {
  x=result.ID;
  }
  }
  }else{
  Serial.println("Error!");}
  //sensor_initialisation();
  //last_DIR = "";
  if (x == 1 && (my_robot.sm_getShape()) != ('o')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.I();
    /*
    front_sensor_status = my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(6);
    left_sensor_status = my_robot.module4_sensor_status(0);
    */
  }
   else if (x == 2 && (my_robot.sm_getShape()) != ('i')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.O();
    /*
    front_sensor_status = my_robot.module1_sensor_status(0);
    right_sensor_status = my_robot.module1_sensor_status(4);
    rear_sensor_status = my_robot.module3_sensor_status(0);
    left_sensor_status = my_robot.module1_sensor_status(10);
    */
  }
  else if (x == 3 && (my_robot.sm_getShape()) != ('l')) {
    my_robot.sm_reset_M1();
    my_robot.sm_reset_M2();
    my_robot.sm_reset_M3();
    my_robot.sm_reset_M4();
    my_robot.L();
  }
   else {
    Serial.println("There is no colour present");
  }


}