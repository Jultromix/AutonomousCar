#include <Arduino.h>
#include <ESP32Servo.h>

//////GPIO PIN DEFINITION//////
//////GPIO PIN DEFINITION//////

// Servo 
const int SERVOPIN = 32;    //Pin 7
Servo servoMotor;

  // DC motor
const int ENA = 33;    // Pin 8 
const int IN1 = 26;    // Pin 9
const int IN2 = 25;    // Pin 10

//////INIT FUNCTIONS//////
//////INIT FUNCTIONS//////

//Servo
void servoInit(){
  Serial.println("Servomotor initiated");
  servoMotor.attach(SERVOPIN);
}

//DC Motor
void dcMotorInit(){
  Serial.println("DC Motor initiated");
  pinMode (ENA, OUTPUT); 
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite(ENA,0);
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

//DC Motor
void setMotorForward(){
  Serial.println("DCMotorForward");
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite(ENA,200);
}
void setMotorBackward(){
  Serial.println("DCMotorBackward");
  digitalWrite (IN2, HIGH);
  digitalWrite (IN1, LOW);
  analogWrite(ENA,200);
}
void setMotorStop(){
  Serial.println("DCMotorStop");
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite(ENA,0);
}


void setup() {
  Serial.begin(115200);
  servoInit();
  dcMotorInit();

}

void loop() {


}