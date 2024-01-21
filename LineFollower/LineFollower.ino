/*
    Line Follower Robot

    
  The objective of this project is to construct a line-following robot, utilizing a reflectance sensor to determine the line position.
  Two motors, managed by an L293D motor driver, are used to guide the robot along the line, aiming to maintain its central position.
  An Arduino UNO serves as the main controller, responsible for reading sensor data, computing the required motor speeds using a PID controller,
  and adjusting the motors accordingly. To address the power limitations of the Arduino,
  an additional 7.4V Li-Po battery is incorporated to power the motors.

  The circuit:

    Input:
      1 x QTR-8A reflectance sensor - pins A0, A1, A2, A3, A4, A5
    
    Output:
      1 x L293D motor controller - pins 4, 5, 6, 7, 10, 11

*/

#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

int m1Speed = 0;
int m2Speed = 0;

// parameters for the pid controller
float kp = 5;
float ki = 0;
float kd = 45;

int p = 1;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 255;
const int calibrationSpeed = 165;

const int errorThreshold = 40;

QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

unsigned long previousMillis = 0;
const long calibrationDuration = 1000;  // 1 second for calibration

void setup() {

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
  delay(500);

  // calibrate the sensors and turn on built in LED during calibration
  digitalWrite(LED_BUILTIN, HIGH);
  calibrateSensors();
  digitalWrite(LED_BUILTIN, LOW);
}
void loop() {
  pidControl(kp, kd, ki);
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {

  // control the motor speeds, negative values make the motors turn backwards, 0 makes the motors stop
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}

void calibrateSensors() {

  unsigned long currentMillis = millis();

  // move the robot left and right to calibrate the sensors
  if (currentMillis - previousMillis < calibrationDuration) {
    for (uint16_t i = 0; i < 100; i++) {
      qtr.calibrate();
      // motor movement during calibration
      if (i < 25 && i > 0) {
        setMotorSpeed(calibrationSpeed, 0);
      } else if (i < 50 && i > 25) {
        setMotorSpeed(-(calibrationSpeed), 0);
      } else if (i < 75 && i > 50) {
        setMotorSpeed(0, calibrationSpeed);
      } else if (i < 100 && i > 75) {
        setMotorSpeed(0, -(calibrationSpeed));
      }
    }
  } else {
    // stop and move to the next step
    setMotorSpeed(0, 0);
    previousMillis = currentMillis;
  }
}

void pidControl(float kp, float kd, float ki) {

  // calculate the motorSpeed based on the error and the PID parameters
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
  p = error;
  i = i + error;
  d = error - lastError;

  int motorSpeed = kp * p + ki * i + kd * d;

  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  // set the needed motor speed
  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  // if the error is very big, the motors turn in oposite directions to allow for tighter turns
  if (error < -errorThreshold || error > errorThreshold) {
    m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
    m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
    // for regular errors the motors turn only forward
  } else {
    m1Speed = constrain(m1Speed, 0, maxSpeed);
    m2Speed = constrain(m2Speed, 0, maxSpeed);
  }

  setMotorSpeed(m1Speed, m2Speed);
}