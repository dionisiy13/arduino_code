#include <Wire.h>
#include <Servo.h> 
#include <Arduino.h>
#include "Car.h"
#include <Wire.h>
#define SLAVE_ADDRESS 0x04


// for distance sensor
int pingPin = 3; // trig
int inPin = 2; // echo

// center
int needToControl = 250;

int number = 0;
int INA = 6; 
int INB = 8;
int EN1 = 11;
int EN = 7;

boolean revers;

double Setpoint, Input, Output;  

Servo myservo; 
Car car;
String inString = "";   

void setup() {
  Serial.begin(9600);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(getData);
 
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  //pinMode (EN1, OUTPUT);
  pinMode (INA, OUTPUT);
  pinMode (INB, OUTPUT);
  pinMode(EN, OUTPUT);   
  digitalWrite(EN, HIGH);
  Input = 250;
  myPID.SetOutputLimits(60, 120);
  myPID.SetMode(AUTOMATIC);
  car.setServo(myservo, 9);
  car.setPins(INA, INB, EN1);
}

void loop() {
    float acd;
    int cm;
    float analog;
    if (getDistance() > 30) {
       car.setSpeed(35);
       car.forward();
    } else {
       car.stopHard(10);
    }
    
}

int getDistance() {
    int cm = sensor.distance();
    return cm;
}

void getData(int d) {
  while(Wire.available()) {
        number = Wire.read();
        Serial.print("data received: ");
        Serial.println(number);
        car.setAngle(number);
        car.setSpeed(35);
        car.forward();
     }
}


