#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 9600

// Pin Definitions
#define HEAD_FRONT_EN
#define HEAD_FRONT_TRIGGER
#define HEAD_FRONT_ECHO

#define HEAD_DOWN_EN
#define HEAD_DOWN_TRIGGER
#define HEAD_DOWN_ECHO

#define HEAD_LEFT_EN
#define HEAD_LEFT_TRIGGER
#define HEAD_LEFT_ECHO

#define HEAD_RIGHT_EN
#define HEAD_RIGHT_TRIGGER
#define HEAD_RIGHT_ECHO

#define MID_FRONT_EN
#define MID_FRONT_TRIGGER
#define MID_FRONT_ECHO

#define MID_DOWN_EN
#define MID_DOWN_TRIGGER
#define MID_DOWN_ECHO

#define MID_LEFT_EN
#define MID_LEFT_TRIGGER
#define MID_LEFT_ECHO

#define MID_RIGHT_EN
#define MID_RIGHT_TRIGGER
#define MID_RIGHT_ECHO

#define TAIL_FRONT_EN
#define TAIL_FRONT_TRIGGER
#define TAIL_FRONT_ECHO

#define TAIL_DOWN_EN
#define TAIL_DOWN_TRIGGER
#define TAIL_DOWN_ECHO

#define TAIL_LEFT_EN
#define TAIL_LEFT_TRIGGER
#define TAIL_LEFT_ECHO

#define TAIL_RIGHT_EN
#define TAIL_RIGHT_TRIGGER
#define TAIL_RIGHT_ECHO

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
