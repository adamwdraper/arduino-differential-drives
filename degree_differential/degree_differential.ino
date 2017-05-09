#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

// pins
const int AileronPin = 5;
const int ElevatorPin = 6;
const int ThrottlePin = 7;

// Padding for throttle
const int Padding = 50;
const int PivotAngle = 25;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - RC differential drive");

  // Set our input pins as such
  pinMode(AileronPin, INPUT);
  pinMode(ElevatorPin, INPUT);  
  pinMode(ThrottlePin, INPUT);

  // create with the default frequency 1.6KHz
  AFMS.begin();
}

void loop() {
  // get pin values
  int aileron = pulseIn(AileronPin, HIGH, 25000);
  int elevator = pulseIn(ElevatorPin, HIGH, 25000);
  int throttle = pulseIn(ThrottlePin, HIGH, 25000);
  double angle;
  int speed;

  // map and constrain throttle and aileron to -255 to 255
  if (_in(aileron, 1000, 2000) && _in(throttle, 1000, 2000)) {
    aileron = map(aileron, 1000, 2000, -500, 500);
    aileron = constrain(aileron, -255, 255);
  
    throttle = map(throttle, 1000, 2000, -500, 500);
    throttle = constrain(throttle, -255, 255);


    // Print debug info
    Serial.print("Aileron:");
    Serial.print(aileron);
    Serial.print(" | Throttle:");
    Serial.print(throttle);

    if (aileron >= 0 && throttle >= 0) {
      // +,+ (0-90˚)
      if (throttle == 0) {
        angle = 0;
      } else if (aileron == 0) {
        angle = 90;
      } else {
        angle = _angle(throttle, aileron);
      }
    } else if (aileron < 0 && throttle >= 0) {
      // -,+ (91-180˚)
      if (throttle == 0) {
        angle = 180;
      } else {
        angle = _angle(throttle, aileron);
      }
    } else if (aileron <= 0 && throttle < 0) {
      // -,- (181-270˚)
      if (aileron == 0) {
        angle = 270;
      } else {
        angle = 360 + _angle(throttle, aileron);
      }
    } else if (aileron > 0 && throttle < 0) {
      // +,- (271-359˚)
      angle = 360 + _angle(throttle, aileron);
    }

    Serial.print(" | Angle:");
    Serial.print(angle);
    
    // find our speed
    speed = max(abs(aileron), abs(throttle));
    
    Serial.print(" | Speed:");
    Serial.print(speed);
    
    // check for action to perform
    if (_in(aileron, -Padding, Padding) && _in(throttle, -Padding, Padding)) {
      // all stop
      stop();
    } else if (_in(angle, 0, PivotAngle) || _in(angle, 360 - PivotAngle, 360) || _in(angle, 180 - PivotAngle, 180 + PivotAngle)) {
      // pivot
      pivot(angle, speed);
    } else {
      // drive
      drive(angle, speed);
    }
  } else {
    Serial.print("Error | Aileron:");
    Serial.print(aileron);
    Serial.print(" | Throttle:");
    Serial.print(throttle);
  }

  Serial.println();
}

// utility to see if value is in a range
boolean _in(int value, int lowerBound, int upperBound) {
  boolean in = false;

  if (value > lowerBound && value < upperBound) {
    in = true;
  }
  
  return in;
}

float _angle(int y, int x) {
  return (atan2(y, x) * 57296) / 1000;
}

void drive(int angle, int speed) {
//  int leftSpeed = speed;
//  int rightSpeed = speed;

  Serial.print(" Drive");

//  if (aileron >= 0 && throttle >= 0) {
//    // +, +
//    // reduce right speed
//    rightSpeed = rightSpeed - aileron;
//
//    // go forward
//    forward(motorLeft, leftSpeed);
//    forward(motorRight, max(rightSpeed, 0));
//  } else if (aileron < 0 && throttle >= 0) {
//    // -, +
//    // reduce left speed
//    leftSpeed = leftSpeed + aileron;
//
//    // go forward
//    forward(motorLeft, max(leftSpeed, 0));
//    forward(motorRight, rightSpeed);    
//  } else if (aileron < 0 && throttle < 0) {
//    // -, -
//    // reduce left speed
//    leftSpeed = leftSpeed - aileron;
//
//    // go backward
//    backward(motorLeft, min(leftSpeed, 0));
//    backward(motorRight, rightSpeed);
//  } else if (aileron >= 0 && throttle <  0) {
//    // +, -
//    // reduce right speed
//    rightSpeed = rightSpeed + aileron;
//
//    // go backward
//    backward(motorLeft, leftSpeed);
//    backward(motorRight, min(rightSpeed, 0));
//  }
}

void pivot(int angle, int speed) {
//  int leftSpeed = aileron;
//  int rightSpeed = aileron;
//
//  if (aileron >= 0) {
//    forward(motorLeft, leftSpeed);
//    backward(motorRight, rightSpeed);
//  } else {
//    backward(motorLeft, leftSpeed);
//    forward(motorRight, rightSpeed);    
//  }

  Serial.print(" Pivot");
}

void forward(Adafruit_DCMotor* motor, int speed) {
  motor->setSpeed(abs(speed));
  motor->run(FORWARD);
}


void backward(Adafruit_DCMotor* motor, int speed) {
  motor->setSpeed(abs(speed));
  motor->run(BACKWARD);
}

// stop w/out break
void stop() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}
