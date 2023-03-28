#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>

// Configuration routines
void setupMotors();
#if SPEED_TESTING
void setSpeed(uint32_t speed);
#endif

// Movements
void moveForward();
void moveReverse();
void moveLeft();
void moveRight();
void moveStop();

#if ROTATE_TESTING
// Rotations
void rotateCW(float degrees);
void rotateCC(float degrees);
#endif

#endif
