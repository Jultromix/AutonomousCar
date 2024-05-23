#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <QuickDebug.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

//======GPIO PIN DEFINITION===========================================================
//======GPIO PIN DEFINITION===========================================================

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

//======CONSTANTS===========================================================
//======CONSTANTS===========================================================

const float soundSpeed20 =  0.034300;   //Speed of sound at 20°C
const float soundSpeed35 =  0.035251;   //Speed of sound at 35°C
const float soundSpeed40 =  0.035554;   //Speed of sound at 40°C

const int sensingDelay = 100;
const int distancesamples = 20;
const int maxDistance = 400;            // Expected max distance to be accurately measured (cm)
const int minDistance = 0;

const uint32_t TiempoEsperaWifi = 5000;
WiFiMulti wifiMulti;

const char* googleMapsApiKey = "AIzaSyA4wkYwQQbMOY4VClZA_Fr01B7Jv55FksE";

//======VARIABLES===========================================================
//======VARIABLES===========================================================

unsigned long timeCounter1;
unsigned long timeCounter2;
long measurements[distancesamples];

double currenttlat;  //current latitude
double currentlng;   //current longitude
double destlatitude = 25.733748;     //example latitude
double destlongitude = -100.214631;   //example longitude
double accuracy;

//======PRPOTOYPE FUNCTIONS===========================================================
//======PRPOTOYPE FUNCTIONS===========================================================

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

void initWiFi(void);
void updateWifi(void);

void getLocation(void);
void getDirections(void);
void parseDirections(String payload);

//======INIT FUNCTIONS===========================================================
//======INIT FUNCTIONS===========================================================

//Servo
void servoInit(){
  Serial.println("Servomotor initiated");
  servoMotor.setPeriodHertz(50); 
  servoMotor.attach(SERVOPIN, 1000, 2000);

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

//======BEHAVIORAL FUNCTIONS===========================================================
//======BEHAVIORAL FUNCTIONS===========================================================

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

void wifiInit() {
  // Replace with your network credentials

  wifiMulti.addAP("Galaxy A33 5GB9A6","tbnb8296");
  wifiMulti.addAP("WLAN_MOSAN","$Mosan1999");
  wifiMulti.addAP("IZZI-8830","ttD7uDeX");

  WiFi.mode(WIFI_MODE_STA);
  Serial.print("Conecting ..");
  while (wifiMulti.run(TiempoEsperaWifi) != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(".. Connected");
  Serial.print("SSID:");
  Serial.print(WiFi.SSID());
  Serial.print(" ID:");
  Serial.println(WiFi.localIP());
}
void updateWifi() {
  if (wifiMulti.run(TiempoEsperaWifi) != WL_CONNECTED) {
    Serial.println("Not connected to Wifi!");
  }
}


void getLocation() {
  if (WiFi.status() == WL_CONNECTED) {
    // Scan for nearby WiFi networks
    int n = WiFi.scanNetworks();
    if (n == 0) {
      Serial.println("No networks found");
      return;
    }

    // Create JSON payload with WiFi data
    JsonDocument jsonDoc;
    JsonArray wifiAccessPoints = jsonDoc["wifiAccessPoints"].to<JsonArray>();

    for (int i = 0; i < n; ++i) {
      JsonObject accessPoint = wifiAccessPoints.add<JsonObject>();
      accessPoint["macAddress"] = WiFi.BSSIDstr(i);
      accessPoint["signalStrength"] = WiFi.RSSI(i);
      // Optional: add "channel" if needed
      // accessPoint["channel"] = WiFi.channel(i);
    }

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Make HTTP POST request
    HTTPClient http;
    String url = String("https://www.googleapis.com/geolocation/v1/geolocate?key=") + googleMapsApiKey;
    http.begin(url.c_str());
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(jsonString);

    if (httpCode > 0) {
      String payload = http.getString();

      // Parse JSON response
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        currenttlat = doc["location"]["lat"];
        currentlng = doc["location"]["lng"] ;
        accuracy = doc["accuracy"];

      } else {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.println("Error on HTTP request");
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void getDirections() {
  if (WiFi.status() == WL_CONNECTED) {
    // Set destination coordinates
    double destLatitude = 37.7749;  // Example latitude of destination
    double destLongitude = -122.4194;  // Example longitude of destination

    // Construct URL for Directions API request
    String url = "https://maps.googleapis.com/maps/api/directions/json?";
    url += "origin=" + String(currenttlat, 7) + "," + String(currentlng, 7);  // Use your current GPS coordinates
    url += "&destination=" + String(destlatitude, 7) + "," + String(destlongitude, 7);
    url += "&key=" + String(googleMapsApiKey);

    Serial.print("Latitude: ");
    Serial.println(currenttlat, 7);
    Serial.print("Longitude: ");
    Serial.println(currentlng, 7);
    Serial.print("Accuracy: ");
    Serial.println(accuracy, 7);
    Serial.print("Destiny Latitude: ");
    Serial.println(destlatitude, 7);
    Serial.print("Destiny Longitude: ");
    Serial.println(destlongitude, 7);

    // Print the constructed URL for debugging
    Serial.println("Request URL: " + url);

    // Make HTTP GET request
    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println("Directions Response: ");
      // Serial.println(payload);
      
      // Parse JSON response to extract directions
      parseDirections(payload);
    } else {
      Serial.println("Error on HTTP request");
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void parseDirections(String payload) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  Serial.println("parseing");
  if (!error) {
    JsonArray steps = doc["routes"][0]["legs"][0]["steps"].as<JsonArray>();
    for (JsonObject step : steps) {
      const char* instruction = step["html_instructions"];
      double distance = step["distance"]["value"];
      double duration = step["duration"]["value"];
      double startLat = step["start_location"]["lat"];
      double startLng = step["start_location"]["lng"];
      double endLat = step["end_location"]["lat"];
      double endLng = step["end_location"]["lng"];

      Serial.println(instruction);
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" meters, Duration: ");
      Serial.print(duration);
      Serial.println(" seconds");

    }
  } else {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
}

//======SETUP===========================================================
//======SETUP===========================================================

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  servoInit();
  dcMotorInit();
  ultrasonicInit();
  wifiInit();
  timeCounter1 = millis();

  getLocation();
  getDirections();
}

//======VOID LOOP===========================================================
//======VOID LOOP===========================================================

void loop() {

  // if (millis() - timeCounter1 > sensingDelay){
    // Execute this lines each "sensingDelay" seconds
    // timeCounter1 = millis();
  // }

  if (millis() - timeCounter1 > 1000){
    // Execute this lines each "sensingDelay" seconds
    timeCounter2 = millis();
    updateWifi(); 
  }
}



