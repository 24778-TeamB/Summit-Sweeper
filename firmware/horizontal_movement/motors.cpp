#include <Arduino.h>
#include "motors.h"
#include "ProjectDef.h"
#include <stdint.h>

uint8_t curSpeed;

void setupMotors()
{
	// Configure right
	pinMode(RIGHT_EN_A, OUTPUT);
	pinMode(RIGHT_EN_B, OUTPUT);
	pinMode(RIGHT_IN1, OUTPUT);
	pinMode(RIGHT_IN2, OUTPUT);
	pinMode(RIGHT_IN3, OUTPUT);
	pinMode(RIGHT_IN4, OUTPUT);

	// Configure Left
	pinMode(LEFT_EN_A, OUTPUT);
	pinMode(LEFT_EN_B, OUTPUT);
	pinMode(LEFT_IN1, OUTPUT);
	pinMode(LEFT_IN2, OUTPUT);
	pinMode(LEFT_IN3, OUTPUT);
	pinMode(LEFT_IN4, OUTPUT);

	// Configure Front
	pinMode(FRONT_EN_A, OUTPUT);
	pinMode(FRONT_EN_B, OUTPUT);
	pinMode(FRONT_IN1, OUTPUT);
	pinMode(FRONT_IN2, OUTPUT);
	pinMode(FRONT_IN3, OUTPUT);
	pinMode(FRONT_IN4, OUTPUT);

  curSpeed = 255;

	// Set initial state
	moveStop();
}

static void updateSpeed()
{
  analogWrite(RIGHT_EN_A, curSpeed);
  analogWrite(RIGHT_EN_B, curSpeed);
  analogWrite(LEFT_EN_A, curSpeed);
  analogWrite(LEFT_EN_B, curSpeed);
}

void setMotorSpeed(uint8_t Speed)
{
  moveStop();

  curSpeed = Speed;
}

void moveForward()
{
  updateSpeed();
  
	digitalWrite(RIGHT_IN1, HIGH);
	digitalWrite(RIGHT_IN2, LOW);
	digitalWrite(RIGHT_IN3, LOW);
	digitalWrite(RIGHT_IN4, HIGH);

	digitalWrite(LEFT_IN3, HIGH);
	digitalWrite(LEFT_IN4, LOW);
	digitalWrite(LEFT_IN1, LOW);
	digitalWrite(LEFT_IN2, HIGH);
}

void moveReverse()
{
  updateSpeed();
  
	digitalWrite(RIGHT_IN1, LOW);
	digitalWrite(RIGHT_IN2, HIGH);
	digitalWrite(RIGHT_IN3, HIGH);
	digitalWrite(RIGHT_IN4, LOW);

	digitalWrite(LEFT_IN3, LOW);
	digitalWrite(LEFT_IN4, HIGH);
	digitalWrite(LEFT_IN1, HIGH);
	digitalWrite(LEFT_IN2, LOW);
}

void moveLeft()
{
  updateSpeed();
  
	digitalWrite(RIGHT_IN1, LOW);
	digitalWrite(RIGHT_IN2, HIGH);
	digitalWrite(LEFT_IN3, LOW);
	digitalWrite(LEFT_IN4, HIGH);

	digitalWrite(RIGHT_IN3, LOW);
	digitalWrite(RIGHT_IN4, HIGH);
	digitalWrite(LEFT_IN1, LOW);
	digitalWrite(LEFT_IN2, HIGH);
}

void moveRight()
{
  updateSpeed();
  
	digitalWrite(RIGHT_IN1, HIGH);
	digitalWrite(RIGHT_IN2, LOW);
	digitalWrite(LEFT_IN3, HIGH);
	digitalWrite(LEFT_IN4, LOW);

	digitalWrite(RIGHT_IN3, HIGH);
	digitalWrite(RIGHT_IN4, LOW);
	digitalWrite(LEFT_IN1, HIGH);
	digitalWrite(LEFT_IN2, LOW);
}

void moveStop()
{
	analogWrite(RIGHT_EN_A, 0);
	analogWrite(RIGHT_EN_B, 0);
	analogWrite(LEFT_EN_A, 0);
	analogWrite(LEFT_EN_B, 0);

	digitalWrite(RIGHT_IN1, LOW);
	digitalWrite(RIGHT_IN2, LOW);
	digitalWrite(RIGHT_IN3, LOW);
	digitalWrite(RIGHT_IN4, LOW);

	digitalWrite(LEFT_IN1, LOW);
	digitalWrite(LEFT_IN2, LOW);
	digitalWrite(LEFT_IN3, LOW);
	digitalWrite(LEFT_IN4, LOW);
}
