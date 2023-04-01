#include <Arduino.h>
#include "ultrasonic.h"
#include "ProjectDef.h"

#define V_SOUND_INV 29.1
#define HIGH_TIME 10
#define LOW_TIME 2

float head_right_avg[SENSOR_BUF_SIZE] = { 0 };
float head_left_avg[SENSOR_BUF_SIZE] = { 0 };
float head_front_avg[SENSOR_BUF_SIZE] = { 0 };
float head_down_avg[SENSOR_BUF_SIZE] = { 0 };

float middle_right_avg[SENSOR_BUF_SIZE] = { 0 };
float middle_left_avg[SENSOR_BUF_SIZE] = { 0 };
float middle_front_avg[SENSOR_BUF_SIZE] = { 0 };
float middle_down_avg[SENSOR_BUF_SIZE] = { 0 };

float tail_right_avg[SENSOR_BUF_SIZE] = { 0 };
float tail_left_avg[SENSOR_BUF_SIZE] = { 0 };
float tail_front_avg[SENSOR_BUF_SIZE] = { 0 };
float tail_down_avg[SENSOR_BUF_SIZE] = { 0 };

uint8_t head_index = 0;
uint8_t middle_index = 0;
uint8_t tail_index = 0;

void init_ultrasonic()
{
	pinMode(HEAD_LEFT_TRIGGER, OUTPUT);
	pinMode(HEAD_LEFT_ECHO, INPUT);

	pinMode(HEAD_RIGHT_TRIGGER, OUTPUT);
	pinMode(HEAD_RIGHT_ECHO, INPUT);

	pinMode(HEAD_FRONT_TRIGGER, OUTPUT);
	pinMode(HEAD_FRONT_ECHO, INPUT);

	pinMode(HEAD_DOWN_TRIGGER, OUTPUT);
	pinMode(HEAD_DOWN_ECHO, INPUT);

	pinMode(MID_LEFT_TRIGGER, OUTPUT);
	pinMode(MID_LEFT_ECHO, INPUT);

	pinMode(MID_RIGHT_TRIGGER, OUTPUT);
	pinMode(MID_RIGHT_ECHO, INPUT);

	pinMode(MID_FRONT_TRIGGER, OUTPUT);
	pinMode(MID_FRONT_ECHO, INPUT);

	pinMode(MID_DOWN_TRIGGER, OUTPUT);
	pinMode(MID_DOWN_ECHO, INPUT);

	pinMode(TAIL_LEFT_TRIGGER, OUTPUT);
	pinMode(TAIL_LEFT_ECHO, INPUT);

	pinMode(TAIL_RIGHT_TRIGGER, OUTPUT);
	pinMode(TAIL_RIGHT_ECHO, INPUT);

	pinMode(TAIL_FRONT_TRIGGER, OUTPUT);
	pinMode(TAIL_FRONT_ECHO, INPUT);

	pinMode(TAIL_DOWN_TRIGGER, OUTPUT);
	pinMode(TAIL_DOWN_ECHO, INPUT);

	digitalWrite(HEAD_LEFT_TRIGGER, LOW);
	digitalWrite(HEAD_RIGHT_TRIGGER, LOW);
	digitalWrite(HEAD_FRONT_TRIGGER, LOW);
	digitalWrite(HEAD_DOWN_TRIGGER, LOW);
	digitalWrite(MID_LEFT_TRIGGER, LOW);
	digitalWrite(MID_RIGHT_TRIGGER, LOW);
	digitalWrite(MID_FRONT_TRIGGER, LOW);
	digitalWrite(MID_DOWN_TRIGGER, LOW);
	digitalWrite(TAIL_LEFT_TRIGGER, LOW);
	digitalWrite(TAIL_RIGHT_TRIGGER, LOW);
	digitalWrite(TAIL_FRONT_TRIGGER, LOW);
	digitalWrite(TAIL_DOWN_TRIGGER, LOW);
}

static float getDistance(uint8_t triggerPin, uint8_t echoPin)
{
	long duration;

	digitalWrite(triggerPin, LOW);
	delayMicroseconds(LOW_TIME);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(HIGH_TIME);
	digitalWrite(triggerPin, LOW);
	duration = pulseIn(echoPin, HIGH);

	return ((float)duration / 2) / V_SOUND_INV;
}

void updateHead()
{
	head_left_avg[head_index] = getDistance(HEAD_LEFT);
	head_right_avg[head_index] = getDistance(HEAD_RIGHT);
	head_down_avg[head_index] = getDistance(HEAD_DOWN);
	head_front_avg[head_index++] = getDistance(HEAD_FRONT);

	if (head_index >= SENSOR_BUF_SIZE)
		head_index = 0;
}

void updateMiddle()
{
	middle_left_avg[middle_index] = getDistance(MID_LEFT);
	middle_right_avg[middle_index] = getDistance(MID_RIGHT);
	middle_down_avg[middle_index] = getDistance(MID_DOWN);
	middle_front_avg[middle_index++] = getDistance(MID_FRONT);

	if (middle_index >= SENSOR_BUF_SIZE)
		middle_index = 0;
}

void updateTail()
{
	tail_left_avg[tail_index] = getDistance(TAIL_LEFT);
	tail_right_avg[tail_index] = getDistance(TAIL_RIGHT);
	tail_down_avg[tail_index] = getDistance(TAIL_DOWN);
	tail_front_avg[tail_index++] = getDistance(TAIL_FRONT);

	if (tail_index >= SENSOR_BUF_SIZE)
		tail_index = 0;
}

ultrasonic_t getLeft()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_LEFT_TRIGGER, HEAD_LEFT_ECHO);
	data.middleModule = getDistance(MID_LEFT_TRIGGER, MID_LEFT_ECHO);
	data.tailModule = getDistance(TAIL_LEFT_TRIGGER, TAIL_LEFT_ECHO);

	return data;
}

ultrasonic_t getRight()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_RIGHT_TRIGGER, HEAD_RIGHT_ECHO);
	data.middleModule = getDistance(MID_RIGHT_TRIGGER, MID_RIGHT_ECHO);
	data.tailModule = getDistance(TAIL_RIGHT_TRIGGER, TAIL_RIGHT_ECHO);

	return data;
}

ultrasonic_t getFront()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_FRONT_TRIGGER, HEAD_FRONT_ECHO);
	data.middleModule = getDistance(MID_FRONT_TRIGGER, MID_FRONT_ECHO);
	data.tailModule = getDistance(TAIL_FRONT_TRIGGER, TAIL_FRONT_ECHO);

	return data;
}

ultrasonic_t getDown()
{
	ultrasonic_t data;
	data.headModule = getDistance(HEAD_DOWN_TRIGGER, HEAD_DOWN_ECHO);
	data.middleModule = getDistance(MID_DOWN_TRIGGER, MID_DOWN_ECHO);
	data.tailModule = getDistance(TAIL_DOWN_TRIGGER, TAIL_DOWN_ECHO);

	return data;
}
