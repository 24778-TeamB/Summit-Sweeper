#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

#include <stdint.h>
#include <Arduino.h>

#define CMD_LINE_BUF_SIZE 256
#define NUMTOKENS 20

#define BAUD 9600

// Pin Definitions

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
