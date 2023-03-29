#ifndef __DISTANCE__
#define __DISTANCE__

typedef struct {
	float frontModule;
	float middleModule;
	float tailModule;
} ultrasonic_t;

void init_ultrasonic();

ultrasonic_t getLeft();
ultrasonic_t getRight();
ultrasonic_t getFront();
ultrasonic_t getDown();

#endif