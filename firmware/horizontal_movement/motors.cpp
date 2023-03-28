#include <Arduino.h>
#include "motors.h"
#include "ProjectDef.h"
#include <stdint.h>

void setupMotors()
{
	// Configure right
	pinMode(RIGHT_EN_A, OUTPUT);
	pinMode(RIGHT_EN_B, OUTPUT);
	pinMode(RIGHT_IN1_A, OUTPUT);
	pinMode(RIGHT_IN2_A, OUTPUT);
	pinMode(RIGHT_IN1_B, OUTPUT);
	pinMode(RIGHT_IN2_B, OUTPUT);

	// Configure Left
	pinMode(LEFT_EN_A, OUTPUT);
	pinMode(LEFT_EN_B, OUTPUT);
	pinMode(LEFT_IN1_A, OUTPUT);
	pinMode(LEFT_IN2_A, OUTPUT);
	pinMode(LEFT_IN1_B, OUTPUT);
	pinMode(LEFT_IN2_B, OUTPUT);

	// Set initial state
	moveStop();
}

void moveForward()
{
	digitalWrite(RIGHT_IN1_A, HIGH);
	digitalWrite(RIGHT_IN2_A, LOW);
	digitalWrite(RIGHT_IN2_A, LOW);
	digitalWrite(RIGHT_IN2_B, HIGH);

	digitalWrite(LEFT_IN1_B, HIGH);
	digitalWrite(LEFT_IN2_B, LOW);
	digitalWrite(LEFT_IN1_A, LOW);
	digitalWrite(LEFT_IN2_A, HIGH);
}

void moveReverse()
{
	digitalWrite(RIGHT_IN1_A, LOW);
	digitalWrite(RIGHT_IN2_A, HIGH);
	digitalWrite(RIGHT_IN2_A, HIGH);
	digitalWrite(RIGHT_IN2_B, LOW);

	digitalWrite(LEFT_IN1_B, LOW);
	digitalWrite(LEFT_IN2_B, HIGH);
	digitalWrite(LEFT_IN1_A, HIGH);
	digitalWrite(LEFT_IN2_A, LOW);
}

void moveLeft()
{
	digitalWrite(RIGHT_IN1_A, HIGH);
	digitalWrite(RIGHT_IN2_A, LOW);
	digitalWrite(LEFT_IN1_B, HIGH);
	digitalWrite(LEFT_IN2_B, LOW);

	digitalWrite(RIGHT_IN1_B, HIGH);
	digitalWrite(RIGHT_IN2_B, LOW);
	digitalWrite(LEFT_IN1_A, HIGH);
	digitalWrite(LEFT_IN2_A, LOW);
}

void moveRight()
{
	digitalWrite(RIGHT_IN1_A, LOW);
	digitalWrite(RIGHT_IN2_A, HIGH);
	digitalWrite(LEFT_IN1_B, LOW);
	digitalWrite(LEFT_IN2_B, HIGH);

	digitalWrite(RIGHT_IN1_B, LOW);
	digitalWrite(RIGHT_IN2_B, HIGH);
	digitalWrite(LEFT_IN1_A, LOW);
	digitalWrite(LEFT_IN2_A, HIGH);
}

void moveStop()
{
	analogWrite(RIGHT_EN_A, 0);
	analogWrite(RIGHT_EN_B, 0);
	analogWrite(LEFT_EN_A, 0);
	analogWrite(LEFT_EN_B, 0);

	digitalWrite(RIGHT_IN1_A, LOW);
	digitalWrite(RIGHT_IN2_A, LOW);
	digitalWrite(RIGHT_IN2_A, LOW);
	digitalWrite(RIGHT_IN2_B, LOW);

	digitalWrite(LEFT_IN1_B, LOW);
	digitalWrite(LEFT_IN2_B, LOW);
	digitalWrite(LEFT_IN1_A, LOW);
	digitalWrite(LEFT_IN2_A, LOW);
}
