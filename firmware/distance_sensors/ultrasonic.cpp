#include <Arduino.h>
#include "ultrasonic.h"
#include "ProjectDef.h"

#define V_SOUND_INV 29.1
#define HIGH_TIME 10
#define INIT_TIME 20

void init_ultrasonic()
{
	pinMode(HEAD_LEFT_EN, OUTPUT);
	pinMode(HEAD_LEFT_TRIGGER, OUTPUT);
	pinMode(HEAD_LEFT_ECHO, INPUT);

	pinMode(HEAD_RIGHT_EN, OUTPUT);
	pinMode(HEAD_RIGHT_TRIGGER, OUTPUT);
	pinMode(HEAD_RIGHT_ECHO, INPUT);

	pinMode(HEAD_FRONT_EN, OUTPUT);
	pinMode(HEAD_FRONT_TRIGGER, OUTPUT);
	pinMode(HEAD_FRONT_ECHO, INPUT);

	pinMode(HEAD_DOWN_EN, OUTPUT);
	pinMode(HEAD_DOWN_TRIGGER, OUTPUT);
	pinMode(HEAD_DOWN_ECHO, INPUT);

	pinMode(MID_LEFT_EN, OUTPUT);
	pinMode(MID_LEFT_TRIGGER, OUTPUT);
	pinMode(MID_LEFT_ECHO, INPUT);

	pinMode(MID_RIGHT_EN, OUTPUT);
	pinMode(MID_RIGHT_TRIGGER, OUTPUT);
	pinMode(MID_RIGHT_ECHO, INPUT);

	pinMode(MID_FRONT_EN, OUTPUT);
	pinMode(MID_FRONT_TRIGGER, OUTPUT);
	pinMode(MID_FRONT_ECHO, INPUT);

	pinMode(MID_DOWN_EN, OUTPUT);
	pinMode(MID_DOWN_TRIGGER, OUTPUT);
	pinMode(MID_DOWN_ECHO, INPUT);

	pinMode(TAIL_LEFT_EN, OUTPUT);
	pinMode(TAIL_LEFT_TRIGGER, OUTPUT);
	pinMode(TAIL_LEFT_ECHO, INPUT);

	pinMode(TAIL_RIGHT_EN, OUTPUT);
	pinMode(TAIL_RIGHT_TRIGGER, OUTPUT);
	pinMode(TAIL_RIGHT_ECHO, INPUT);

	pinMode(TAIL_FRONT_EN, OUTPUT);
	pinMode(TAIL_FRONT_TRIGGER, OUTPUT);
	pinMode(TAIL_FRONT_ECHO, INPUT);

	pinMode(TAIL_DOWN_EN, OUTPUT);
	pinMode(TAIL_DOWN_TRIGGER, OUTPUT);
	pinMode(TAIL_DOWN_ECHO, INPUT);

	digitalWrite(HEAD_LEFT_EN, LOW);
	digitalWrite(HEAD_LEFT_TRIGGER, LOW);

	digitalWrite(HEAD_RIGHT_EN, LOW);
	digitalWrite(HEAD_RIGHT_TRIGGER, LOW);

	digitalWrite(HEAD_FRONT_EN, LOW);
	digitalWrite(HEAD_FRONT_TRIGGER, LOW);

	digitalWrite(HEAD_DOWN_EN, LOW);
	digitalWrite(HEAD_DOWN_TRIGGER, LOW);

	digitalWrite(MID_LEFT_EN, LOW);
	digitalWrite(MID_LEFT_TRIGGER, LOW);

	digitalWrite(MID_RIGHT_EN, LOW);
	digitalWrite(MID_RIGHT_TRIGGER, LOW);

	digitalWrite(MID_FRONT_EN, LOW);
	digitalWrite(MID_FRONT_TRIGGER, LOW);

	digitalWrite(MID_DOWN_EN, LOW);
	digitalWrite(MID_DOWN_TRIGGER, LOW);

	digitalWrite(TAIL_LEFT_EN, LOW);
	digitalWrite(TAIL_LEFT_TRIGGER, LOW);

	digitalWrite(TAIL_RIGHT_EN, LOW);
	digitalWrite(TAIL_RIGHT_TRIGGER, LOW);

	digitalWrite(TAIL_FRONT_EN, LOW);
	digitalWrite(TAIL_FRONT_TRIGGER, LOW);

	digitalWrite(TAIL_DOWN_EN, LOW);
	digitalWrite(TAIL_DOWN_TRIGGER, LOW);
}

static float getDistance(uint8_t enablePin, uint8_t triggerPin, uint8_t echoPin)
{
	long duration;

	digitalWrite(enablePin, HIGH);  // Turn on sensor
	delayMicroSeconds(INIT_TIME);  // wait for sensor to turn on
	digitalWrite(triggerPin, HIGH);
	delayMicroSeconds(HIGH_TIME);
	digitalWrite(triggerPin, LOW);
	duration = pulseIn(echoPin, HIGH);
	digitalWrite(enablePin, LOW);  // Turn off sensor

	return ((float)duration / 2) / V_SOUND_INV;
}

ultrasonic_t getLeft()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_LEFT_EN, HEAD_LEFT_TRIGGER, HEAD_LEFT_ECHO);
	data.middleModule = getDistance(MID_LEFT_EN, MID_LEFT_TRIGGER, MID_LEFT_ECHO);
	data.tailModule = getDistance(TAIL_LEFT_EN, TAIL_LEFT_TRIGGER, TAIL_LEFT_ECHO);

	return data;
}

ultrasonic_t getRight()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_RIGHT_EN, HEAD_RIGHT_TRIGGER, HEAD_RIGHT_ECHO);
	data.middleModule = getDistance(MID_RIGHT_EN, MID_RIGHT_TRIGGER, MID_RIGHT_ECHO);
	data.tailModule = getDistance(TAIL_RIGHT_EN, TAIL_RIGHT_TRIGGER, TAIL_RIGHT_ECHO);

	return data;
}

ultrasonic_t getFront()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_FRONT_EN, HEAD_FRONT_TRIGGER, HEAD_FRONT_ECHO);
	data.middleModule = getDistance(MID_FRONT_EN, MID_FRONT_TRIGGER, MID_FRONT_ECHO);
	data.tailModule = getDistance(TAIL_FRONT_EN, TAIL_FRONT_TRIGGER, TAIL_FRONT_ECHO);

	return data;
}

ultrasonic_t getDown()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_DOWN_EN, HEAD_DOWN_TRIGGER, HEAD_DOWN_ECHO);
	data.middleModule = getDistance(MID_DOWN_EN, MID_DOWN_TRIGGER, MID_DOWN_ECHO);
	data.tailModule = getDistance(TAIL_DOWN_EN, TAIL_DOWN_TRIGGER, TAIL_DOWN_ECHO);

	return data;
}