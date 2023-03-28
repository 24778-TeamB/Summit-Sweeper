#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 9600

// Pin Definitions
#define RIGHT_EN_A 3
#define RIGHT_IN1_A 4
#define RIGHT_IN2_A 7

#define RIGHT_EN_B 9
#define RIGHT_IN1_B 2
#define RIGHT_IN2_B 8

#define LEFT_EN_A 5
#define LEFT_IN1_A 24
#define LEFT_IN2_A 25

#define LEFT_EN_B 10
#define LEFT_IN1_B 26
#define LEFT_IN2_B 27

// Events
#define E_NO_EVENT 0x00000000
#define E_SERIAL_ACTIVE 0x00000001
#define E_VALID_MASK (E_SERIAL_ACTIVE)

const char projectClass[] = "Carnegie Mellon Mechatronics";
const char projectName[] = "Horizontal Movement";
const char projectTeam[] = "Team B";

#define MAJOR_VER 0
#define MINOR_VER 0
#define BUILD 0

extern volatile uint32_t gEvents;

#endif // !PROJECT_DEF_H
