#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 115200

#define SENSOR_BUF_SIZE 64

// Pin Definitions
// Pin Definitions
#define CENTER_RIGHT 5
#define CENTER_LEFT 6
#define REAR_RIGHT 2
#define REAR_LEFT 3
#define SIDE_RIGHT 4
#define SIDE_LEFT 7

#define CENTER CENTER_RIGHT, CENTER_LEFT
#define REAR REAR_RIGHT, REAR_LEFT
#define SIDE SIDE_RIGHT, SIDE_LEFT

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

#endif // !PROJECT_DEF_H
