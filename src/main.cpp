#include <Arduino.h>
#include <ESP32Servo.h>

//////GPIO PIN DEFINITION//////
//////GPIO PIN DEFINITION//////

// Servo 
const int SERVOPIN = 32;    //Pin 7
Servo servoMotor;

// DC motor
const int ENA = 33;     // Pin 8 
const int IN1 = 26;     // Pin 9
const int IN2 = 25;     // Pin 10

// Ultrasonic
const int ECHO = 27;    //Pin 11
const int TRIG = 14;    //Pin 12


//////CONSTANTS//////
//////CONSTANTS//////
const float soundSpeed20 =  0.034300;  //Speed of sound at 20°C
const float soundSpeed35 =  0.035251;  //Speed of sound at 35°C
const float soundSpeed40 =  0.035554;  //Speed of sound at 40°C

const int sensingDelay = 100;
const int distancesamples = 10;

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
long getDistance(void);

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
    Serial.println("Ultrasonic initiated");
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
long getDistance(){
  // Clear the trigPin
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // Set the trigPin on HIGH state for 10 microseconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read many samples of duration of the wave (microseconds) and append them to an array 
  for (int i = 0; i < distancesamples; i++) {
    measurements[i] = pulseIn(ECHO, HIGH);
    delay(2); //Small pause between measurements
  }

  // Gets the median of wave travel duration
  long duration = median(measurements, distancesamples);
  int distance = duration * soundSpeed40 / 2; // Convert duration to distance (cm)
  return distance;
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


void setup() {
  Serial.begin(115200);
  servoInit();
  dcMotorInit();
  ultrasonicInit();

  timeCounter1 = millis();
}

void loop() {

  if (millis() - timeCounter1 >= sensingDelay){
    timeCounter1 = millis();
    // Run the next instructions each "sensingDelay" miliseconds
    Serial.println(getDistance());
  }
  
}