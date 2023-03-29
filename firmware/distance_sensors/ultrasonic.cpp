#include <Arduino.h>
#include "ultrasonic.h"
#include "ProjectDef.h"

#define V_SOUND_INV 29.1
#define HIGH_TIME 10

void init_ultrasonic()
{

}

static float getDistance(uint8_t enablePin, uint8_t triggerPin, uint8_t echoPin)
{
	long duration;

	digitalWrite(enablePin, HIGH);  // Turn on sensor
	delayMicroSeconds(20);  // wait for sensor to turn on
	digitalWrite(triggerPin, HIGH);
	delayMicroSeconds(HIGH_TIME);
	digitalWrite(triggerPin, LOW);
	duration = pulseIn(echoPin, HIGH);

	return ((float)duration / 2) / V_SOUND_INV;
}
