#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdbool.h>
#include <stdint.h>

#define RIGHT_1 0x00
#define RIGHT_2 0x01
#define RIGHT_3 0x02
#define LEFT_1 0x10
#define LEFT_2 0x11
#define LEFT_3 0x12

// Configuration routines
void setupMotors();
bool setMotorSpeed(uint8_t motor, uint8_t Speed);

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
