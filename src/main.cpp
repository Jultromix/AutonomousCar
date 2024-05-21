#include <Arduino.h>
#include <ESP32Servo.h>
// #include <TinyGPS++.h>
#include <TinyGPSPlus.h>

//////GPIO PIN DEFINITION//////
//////GPIO PIN DEFINITION//////

// Servo 
const int SERVOPIN = 32;    //Pin 7 (read as an IC)
Servo servoMotor;

// DC motor
const int ENA = 33;     // Pin 8  (read as an IC)
const int IN1 = 26;     // Pin 9 (read as an IC)
const int IN2 = 25;     // Pin 10 (read as an IC)

// Ultrasonic
const int ECHO = 27;    //Pin 11 (read as an IC)
const int TRIG = 14;    //Pin 12 (read as an IC)

// GPS
TinyGPSPlus gps;

//////CONSTANTS//////
//////CONSTANTS//////
const float soundSpeed20 =  0.034300;   //Speed of sound at 20°C
const float soundSpeed35 =  0.035251;   //Speed of sound at 35°C
const float soundSpeed40 =  0.035554;   //Speed of sound at 40°C

const int sensingDelay = 100;
const int distancesamples = 20;
const int maxDistance = 400;            // Expected max distance to be accurately measured (cm
const int minDistance = 0;

//////VARIABLES//////
//////VARIABLES//////
unsigned long timeCounter1;

long measurements[distancesamples];

//////PRPOTOYPE FUNCTIONS//////
//////PROTOTYPE FUNCTIONS//////
void servoInit(void);
void setServoRight(void);
void setServoLeft(void);
void setServoMiddle(void);

void dcMotorInit(void);
void setMotorForward(void);
void setMotorStop(void);
void setMotorBackward(void);

void avoidInminentCollition(void);
void avoidDistantCollition(void);

void ultrasonicInit(void);
void bubbleSort(long arr[], int size);
long median(long arr[], int size);
long calculateDistance(void);
long measureDistance(void);

void displayLocation(void);


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

// Ultrasonic
void ultrasonicInit(){
  Serial.println("Ultrasonic initiated\n");
  pinMode(TRIG, OUTPUT); 
  pinMode(ECHO, INPUT);  
  digitalWrite(TRIG, LOW);  //Inicializamos el pin con 0
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

//Obstacle avoidance
void avoidInminentCollition(){
  Serial.println("Inminent obstacle");
  setMotorStop();
  delay(2000);

  setMotorBackward();
  delay(2500);
}
void avoidDistantCollition(){
  Serial.println("Distant obstacle");
  setServoLeft();
  delay(2000);

  setServoMiddle();
  delay(1000);

  setServoRight();
  delay(1500);

  setServoMiddle();
}

//Distance Measurement
long calculatetDistance(){
  // Read many samples of duration of the wave (microseconds) and append them to an array 
  for (int i = 0; i < distancesamples; i++) {
    long duration = measureDistance();
    int distance = duration * soundSpeed40 / 2;

    // distance must 
    if (distance >= minDistance && distance <= maxDistance){
      measurements[i] = duration;
      delay(2); //Small pause between measurements
    }
 }

  // Gets the median of wave travel duration
  long medianDuration = median(measurements, distancesamples);
  int filteredDistance = medianDuration * soundSpeed40 / 2; // Convert duration to distance (cm)
  return filteredDistance;
}
long measureDistance(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration;
}
// gets the median of an array of samples
long median(long arr[], int size){
  bubbleSort(arr, size);
  if (size % 2 == 0) {
    return (arr[size / 2 - 1] + arr[size / 2]) / 2;
  } else {
    return arr[size / 2];
  }
}
// Sorts the array of samples
void bubbleSort(long arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        long temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}


void displayLocation(){
  if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) { 
      Serial.print(F("Location: "));
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print("Longitude: ");
        Serial.print(gps.location.lng(), 6);
        Serial.println();
      }else if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      }else {
        Serial.println(F("INVALID"));
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  servoInit();
  dcMotorInit();
  ultrasonicInit();
  
  timeCounter1 = millis();
}

void loop() {

  // if (millis() - timeCounter1 > sensingDelay){
  //   timeCounter1 = millis();
  //   // Serial.println(calculatetDistance());  //Prints the current distance
  //   displayLocation(); 
  // }
  displayLocation();

}