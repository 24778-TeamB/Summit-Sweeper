#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 9600

// Pin Definitions
#define HEAD_FRONT_TRIGGER 6
#define HEAD_FRONT_ECHO 7

#define HEAD_DOWN_TRIGGER 6
#define HEAD_DOWN_ECHO 7

#define HEAD_LEFT_TRIGGER 6
#define HEAD_LEFT_ECHO 7

#define HEAD_RIGHT_TRIGGER 6
#define HEAD_RIGHT_ECHO 7

#define MID_FRONT_TRIGGER 6
#define MID_FRONT_ECHO 7

#define MID_DOWN_TRIGGER 6
#define MID_DOWN_ECHO 7

#define MID_LEFT_TRIGGER 6
#define MID_LEFT_ECHO 7

#define MID_RIGHT_TRIGGER 6
#define MID_RIGHT_ECHO 7

#define TAIL_FRONT_TRIGGER 6
#define TAIL_FRONT_ECHO 7

#define TAIL_DOWN_TRIGGER 6
#define TAIL_DOWN_ECHO 7

#define TAIL_LEFT_TRIGGER 6
#define TAIL_LEFT_ECHO 7

#define TAIL_RIGHT_TRIGGER 6
#define TAIL_RIGHT_ECHO 7

// Events
#define E_NO_EVENT 0x00000000
#define E_SERIAL_ACTIVE 0x00000001
#define E_VALID_MASK (E_SERIAL_ACTIVE)

const char projectClass[] = "Carnegie Mellon Mechatronics";
const char projectName[] = "Distance Sensors";
const char projectTeam[] = "Team B";

#define MAJOR_VER 0
#define MINOR_VER 0
#define BUILD 0

extern volatile uint32_t gEvents;

#endif // !PROJECT_DEF_H
