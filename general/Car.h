#include <Arduino.h>
class Car
{
  int IN1, IN2, EN1;
  Servo servo;
  public:
    void setPins(int, int, int);
    void stopEasy();
    void stopHard(int);
    void backward();
    void forward();
    void setSpeed(int);
    void setAngle(int);
    void setServo(Servo, int);
    void detachServo();
};

void Car::setPins(int x, int y, int z)
{
  IN1 = x;
  IN2 = y;
  EN1 = z;
}

void Car::setServo(Servo turning, int pin)
{
  servo = turning;
  servo.attach(pin);
}

void Car::detachServo()
{
   servo.detach();
}

void Car::setAngle(int pos) 
{
  servo.write(pos);
  delay(15);
}

void Car::backward(){
    //Serial.println("backward");
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
}


void Car::stopHard(int pause = 0){
    Serial.println("stop h");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    delay(pause);
 
}

void Car::stopEasy(){
    Serial.println("stop e");
    setSpeed(0);
}

void Car::forward(){
    //Serial.println("forward");
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, HIGH);
}

void Car::setSpeed(int speed) {
    analogWrite(EN1, speed);
}


