#include <Arduino.h>
#include <ESP32Servo.h>

//////GPIO PIN DEFINITION//////
//////GPIO PIN DEFINITION//////

// Servo 
const int SERVOPIN = 33;    //Pin 8
Servo servoMotor;

//////INIT FUNCTIONS//////
//////INIT FUNCTIONS//////

//Servo
void servoInit(){
  Serial.println("Servomotor initiated");
  servoMotor.attach(SERVOPIN);

}

//////BEHAVIORAL FUNCTIONS//////
//////BEHAVIORAL FUNCTIONS//////

//Servo
void setServoRight(){
  //for debugging
  Serial.println("ServoRight");
  servoMotor.write(20);
}
void setServoLeft(){
  //for debugging
  Serial.println("ServoLeft");
  servoMotor.write(135);
}
void setServoMiddle(){
  //for debugging
  Serial.println("ServoMiddle");
  servoMotor.write(80);
}


void setup() {
  Serial.begin(115200);
  servoInit();

}

void loop() {


}