#ifndef __DISTANCE__
#define __DISTANCE__

#if 0

typedef struct {
	float headModule;
	float middleModule;
	float tailModule;
} ultrasonic_t;

void init_ultrasonic();

void updateHead();
void updateMiddle();
void updateTail();

ultrasonic_t getLeft();
ultrasonic_t getRight();
ultrasonic_t getFront();
ultrasonic_t getDown();

#endif

#endif