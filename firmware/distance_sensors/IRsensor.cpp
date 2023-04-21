#include <Arduino.h>
#include "ProjectDef.h"

void initIR(void)
{
	pinMode(CENTER_RIGHT, INPUT);
	pinMode(CENTER_LEFT, INPUT);

	pinMode(REAR_RIGHT, INPUT);
	pinMode(REAR_LEFT, INPUT);

	pinMode(SIDE_RIGHT, INPUT);
	pinMode(SIDE_LEFT, INPUT);
}

void get_IR_readings(uint8_t right, uint8_t left, uint8_t *rightReading, uint8_t *leftReading)
{
	*rightReading = digitalRead(right);
	*leftReading = digitalRead(left);
}
