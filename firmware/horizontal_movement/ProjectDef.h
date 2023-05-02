#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <Arduino.h>
#include <stdint.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS         20

#define BAUD              115200

// Pin Definitions
#define RIGHT_EN_A 4
#define RIGHT_EN_B 5
#define RIGHT_IN1  26
#define RIGHT_IN2  27
#define RIGHT_IN3  28
#define RIGHT_IN4  29

#define LEFT_EN_A  3
#define LEFT_EN_B  2
#define LEFT_IN1   23
#define LEFT_IN2   22
#define LEFT_IN3   24
#define LEFT_IN4   25

#define FRONT_EN_A 8
#define FRONT_EN_B 7
#define FRONT_IN1  32
#define FRONT_IN2  33
#define FRONT_IN3  30
#define FRONT_IN4  31

#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define MID_RIGHT 2
#define MID_LEFT 3
#define REAR_RIGHT 4
#define REAR_LEFT 5

#define NUM_MOTORS 6

#define MOVE_STOP 0
#define MOVE_RIGHT 1
#define MOVE_LEFT 2
#define MOVE_FORWARD 3
#define MOVE_REVERSE 4

// Events
#define E_NO_EVENT      0x00000000
#define E_SERIAL_ACTIVE 0x00000001
#define E_VALID_MASK    (E_SERIAL_ACTIVE)

const char projectClass[] = "Carnegie Mellon Mechatronics";
const char projectName[] = "Horizontal Movement";
const char projectTeam[] = "Team B";

#define MAJOR_VER 0
#define MINOR_VER 0
#define BUILD     0

#endif // !PROJECT_DEF_H
