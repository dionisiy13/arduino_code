#include <Wire.h>
#include <Servo.h> 
#include <iarduino_HC_SR04_int.h>
#include "Car.h"
#include <PID_v1.h>

// for distance sensor
int pingPin = 3; // trig
int inPin = 2; // echo

// center
int needToControl = 400;

// for motors
int IN1 = 8; 
int IN2 = 11;
int EN1 = 6;

boolean revers;

double Setpoint, Input, Output;
iarduino_HC_SR04_int sensor(pingPin,inPin);   

Servo myservo; 
Car car;
String inString = "";   

PID myPID(&Input, &Output, &Setpoint,1,0.5,1, DIRECT);

void setup() {
  Serial.begin(9600);
  
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  //pinMode (EN1, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  Input = 400;

  myPID.SetOutputLimits(65, 120);
  if (Setpoint < Input){
    revers=true;
    myPID.SetControllerDirection(REVERSE);
  }
  myPID.SetMode(AUTOMATIC);
  
  car.setPins(IN1, IN2, EN1);
  car.setServo(myservo, 10);
   digitalWrite (IN2, HIGH);
  digitalWrite (IN2, LOW); 
  
}

int acd, analog, cm;

void loop() {

  if (getDistance() > 150) {
      car.forward();
      getData(150);
  } else if (getDistance() < 150 && getDistance() > 30 ) {
      acd = 150 - cm; 
      analog = 150 - (150 * (acd / 150));
      acd = (int) analog;
      car.forward();
      getData(acd);
  } else {
      car.stopHard(200);
      car.setSpeed(0);
      car.forward();
   }
  
  getData(210);
}

int getDistance() {
    int cm = sensor.distance();
    return cm;
}

void getData(int speed) {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      Setpoint = inString.toInt();
      myPID.Compute();
      if (revers) {
         car.setAngle(120-Output);
      } else {
        car.setAngle(Output);
      }
      Serial.print(Output);
      
      inString = "";
    }
  }
}


