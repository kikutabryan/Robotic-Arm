#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BasicOTA.hpp"

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50

void setup()
{
  BasicOTA::setupBasicOTA(); // Set up the over the air communication
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(27000000);
  servoDriver.setPWMFreq(SERVO_FREQ);
}

void loop()
{
  BasicOTA::handleBasicOTA(); // Handle attempts to upload new code
  servoDriver.writeMicroseconds(0, 1500);
  servoDriver.writeMicroseconds(1, 1500);
  servoDriver.writeMicroseconds(2, 1500);
}