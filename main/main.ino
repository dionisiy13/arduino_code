#include <Wire.h>
#include <Servo.h> 
#include <PID_v1.h>
#include "Car.h"
//#include <iarduino_HC_SR04_int.h>


// for distance sensor
int pingPin = 3; // trig
int inPin = 2; // echo

// center
int needToControl = 400;

// for motors
int INA = 6; 
int INB = 8;
int EN1 = 11;
int EN = 7;

boolean revers;

double Setpoint, Input, Output;
//iarduino_HC_SR04_int sensor(pingPin,inPin);   

Servo myservo; 
Car car;
String inString = "";   

PID myPID(&Input, &Output, &Setpoint,2,0.5,1, DIRECT);

void setup() {
  Serial.begin(9600);
  
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  //pinMode (EN1, OUTPUT);
  pinMode (INA, OUTPUT);
  pinMode (INB, OUTPUT);
  pinMode(EN, OUTPUT);   
  digitalWrite(EN, HIGH);
  
  Input = 400;

  myPID.SetOutputLimits(65, 120);
  if (Setpoint < Input){
    revers=true;
    myPID.SetControllerDirection(REVERSE);
  }
  myPID.SetMode(AUTOMATIC);
  
  car.setPins(INA, INB, EN1);
  car.setServo(myservo, 10);
   digitalWrite (INA, HIGH);
  digitalWrite (INB, LOW); 
  
}

void loop() {
  float acd;
  int cm;
  float analog;
  if (getDistance() > 150) {
      car.forward();
      car.setSpeed(40);
      getData();
  } else if (getDistance() < 150 && getDistance() > 30 ) {
      acd = 150 - cm; 
      analog = 150 - (150 * (acd / 150));
      acd = (int) analog;
      car.forward();
      car.setSpeed(40);
      getData();
  } else {
      car.stopHard(200);
      car.setSpeed(40);
      car.forward();
   }
  
  getData();
}

int getDistance() {
    return 160;
    //int cm = sensor.distance();
    //return cm;
}

void getData() {
  if (Serial.available()) {
	  car.setSpeed(0);
  } 

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
      //detectWall();
      Setpoint = inString.toInt();
      myPID.Compute();
      if (revers) {
         car.setAngle(120-Output);
      } else {
        car.setAngle(Output);
      }
      
      inString = "";
    }
  }
}


