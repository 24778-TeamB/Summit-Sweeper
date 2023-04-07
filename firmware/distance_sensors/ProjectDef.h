#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 115200

#define SENSOR_BUF_SIZE 64

// Pin Definitions
#define HEAD_FRONT_TRIGGER 52
#define HEAD_FRONT_ECHO 53

#define HEAD_DOWN_TRIGGER 52
#define HEAD_DOWN_ECHO 53

#define HEAD_LEFT_TRIGGER 52
#define HEAD_LEFT_ECHO 53

#define HEAD_RIGHT_TRIGGER 52
#define HEAD_RIGHT_ECHO 53

#define MID_FRONT_TRIGGER 52
#define MID_FRONT_ECHO 53

#define MID_DOWN_TRIGGER 52
#define MID_DOWN_ECHO 53

#define MID_LEFT_TRIGGER 52
#define MID_LEFT_ECHO 53

#define MID_RIGHT_TRIGGER 52
#define MID_RIGHT_ECHO 53

#define TAIL_FRONT_TRIGGER 52
#define TAIL_FRONT_ECHO 53

#define TAIL_DOWN_TRIGGER 52
#define TAIL_DOWN_ECHO 53

#define TAIL_LEFT_TRIGGER 52
#define TAIL_LEFT_ECHO 53

#define TAIL_RIGHT_TRIGGER 52
#define TAIL_RIGHT_ECHO 53

#define HEAD_FRONT HEAD_FRONT_TRIGGER, HEAD_FRONT_ECHO
#define HEAD_DOWN HEAD_DOWN_TRIGGER, HEAD_DOWN_ECHO
#define HEAD_LEFT HEAD_LEFT_TRIGGER, HEAD_LEFT_ECHO
#define HEAD_RIGHT HEAD_RIGHT_TRIGGER, HEAD_RIGHT_ECHO

#define MID_FRONT MID_FRONT_TRIGGER, MID_FRONT_ECHO
#define MID_DOWN MID_DOWN_TRIGGER, MID_DOWN_ECHO
#define MID_LEFT MID_LEFT_TRIGGER, MID_LEFT_ECHO
#define MID_RIGHT MID_RIGHT_TRIGGER, MID_RIGHT_ECHO

#define TAIL_FRONT TAIL_FRONT_TRIGGER, TAIL_FRONT_ECHO
#define TAIL_DOWN TAIL_DOWN_TRIGGER, TAIL_DOWN_ECHO
#define TAIL_LEFT TAIL_LEFT_TRIGGER, TAIL_LEFT_ECHO
#define TAIL_RIGHT TAIL_RIGHT_TRIGGER, TAIL_RIGHT_ECHO

// Events
#define E_NO_EVENT 0x00000000
#define E_SERIAL_ACTIVE 0x00000001
#define E_TIMER1 0x00000002
#define E_VALID_MASK (E_TIMER1 | E_SERIAL_ACTIVE)

const char projectClass[] = "Carnegie Mellon Mechatronics";
const char projectName[] = "Distance Sensors";
const char projectTeam[] = "Team B";

#define MAJOR_VER 0
#define MINOR_VER 0
#define BUILD 0

extern volatile uint32_t gEvents;

#endif // !PROJECT_DEF_H
