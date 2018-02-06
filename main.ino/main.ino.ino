#include <Wire.h>
#include <Servo.h> 
#include <iarduino_HC_SR04_int.h>
#include "Car.h"
#include <PID_v1.h>//библиотека ПИД-ругулятора

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
String inString = "";    // string to hold 

PID myPID(&Input, &Output, &Setpoint,8.6,0.5,1, DIRECT);//создаем ПИД-регулятор

void setup() {
  Serial.begin(9600);
  
  pinMode(inPin, INPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  pinMode (EN1, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  Setpoint = 400;//заданная температура в салоне автомобиля
  myPID.SetOutputLimits(65, 120);//устанавливаем границы выходного сигнала для ПИД-регулятора
  if (Setpoint < Input){//если начальная температура больше заданной
    revers=true;
    myPID.SetControllerDirection(REVERSE);//ПИД-регулятор используем обратный
  }
  myPID.SetMode(AUTOMATIC);//включаем ПИД-регулятор
  
  car.setPins(IN1, IN2, EN1);
  car.setServo(myservo, 10);
  
}

void loop() {
  getData();
}

void getData() {
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

      Input = inString.toInt();//анализируем температуру салона
      myPID.Compute();//считаем выходной сигнал ПИД-регулятора
      if (revers)//если пид регулятор обратный, то сервой управляем также относительно противоположной крайней точки
         car.setAngle(120-Output);
      else
      car.setAngle(Output);
      myservo.detach();//отключаемся от сервы
      car.setSpeed(100);
      
      // clear the string for new input:
      inString = "";
    }
  }
}


