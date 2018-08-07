#include <Wire.h>
#include <Servo.h> 
#include <Arduino.h>
#include <PID_v1.h>
#include "Car.h"


// for distance sensor
int pingPin = 3; // trig
int inPin = 2; // echo

// center
int needToControl = 250;


int INA = 6; 
int INB = 8;
int EN1 = 11;
int EN = 7;

boolean revers;

double Setpoint, Input, Output;  

Servo myservo; 
Car car;
String inString = "";   

PID myPID(&Input, &Output, &Setpoint,1,0,0, DIRECT);

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
  
  Input = 250;
  


  myPID.SetOutputLimits(60, 120);
  myPID.SetMode(AUTOMATIC);
  
  car.setPins(INA, INB, EN1);
  car.setServo(myservo, 10);
  
}

void loop() {
  float acd;
  int cm;
  float analog;
   
   if (getDistance() > 150) {
        car.forward();
        car.setSpeed(55);
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
  
}

int getDistance() {
    return 160;
    //int cm = sensor.distance();
    //return cm;
}

void getData() {

  //while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      //erial.print("Value:");
      //Serial.println(inString.toInt());
      //Serial.print("String: ");
      //Serial.println(inString);
      //detectWall();
      Setpoint = inString.toInt();
      int value;
      car.setAngle(Setpoint);
      
     
      
      inString = "";
    }
  //}

}


