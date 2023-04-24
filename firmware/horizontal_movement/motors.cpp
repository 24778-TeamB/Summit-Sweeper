#include <Arduino.h>
#include "motors.h"
#include "ProjectDef.h"
#include <stdint.h>

uint8_t left1_speed, left2_speed, left3_speed, right1_speed, right2_speed, right3_speed;

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

 right1_speed = 255;
 right2_speed = 255;
 right3_speed = 255;
 left1_speed = 255;
 left2_speed = 255;
 left3_speed = 255;
 
	// Set initial state
	moveStop();
}

static void updateSpeed()
{
  analogWrite(RIGHT_EN_A, right1_speed);
  analogWrite(RIGHT_EN_B, right2_speed);
  analogWrite(LEFT_EN_B, left1_speed);
  analogWrite(LEFT_EN_A, left2_speed);
  analogWrite(FRONT_EN_B, right3_speed);
  analogWrite(FRONT_EN_A, left3_speed);
}

bool setMotorSpeed(uint8_t motor, uint8_t Speed)
{
  bool success = true;
  moveStop();

  switch(motor)
  {
    case RIGHT_1: right1_speed = Speed;
      break;
    case RIGHT_2: right2_speed = Speed;
      break;
    case RIGHT_3: right3_speed = Speed;
      break;
    case LEFT_1: left1_speed = Speed;
      break;
    case LEFT_2: left2_speed = Speed;
      break;
    case LEFT_3: left3_speed = Speed;
      break;
    default: success = false;
  }

  return success;
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

	digitalWrite(FRONT_IN1, HIGH);
	digitalWrite(FRONT_IN2, LOW);
	digitalWrite(FRONT_IN3, HIGH);
	digitalWrite(FRONT_IN4, LOW);
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

	digitalWrite(FRONT_IN1, LOW);
	digitalWrite(FRONT_IN2, HIGH);
	digitalWrite(FRONT_IN3, LOW);
	digitalWrite(FRONT_IN4, HIGH);
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

  analogWrite(FRONT_EN_A, 0);
  analogWrite(FRONT_EN_B, 0);
  digitalWrite(FRONT_IN1, LOW);
  digitalWrite(FRONT_IN2, LOW);
  digitalWrite(FRONT_IN3, LOW);
  digitalWrite(FRONT_IN4, LOW);
  
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
	analogWrite(FRONT_EN_A, 0);
	analogWrite(FRONT_EN_B, 0);

	digitalWrite(FRONT_IN1, LOW);
	digitalWrite(FRONT_IN2, LOW);
	digitalWrite(FRONT_IN3, LOW);
	digitalWrite(FRONT_IN4, LOW);

	digitalWrite(RIGHT_IN1, LOW);
	digitalWrite(RIGHT_IN2, LOW);
	digitalWrite(RIGHT_IN3, LOW);
	digitalWrite(RIGHT_IN4, LOW);

	digitalWrite(LEFT_IN1, LOW);
	digitalWrite(LEFT_IN2, LOW);
	digitalWrite(LEFT_IN3, LOW);
	digitalWrite(LEFT_IN4, LOW);
}
