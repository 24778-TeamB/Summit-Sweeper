#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

struct cmd_str
{
	uint16_t cmd;
	int8_t(*ftn)(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);
};

extern const struct cmd_str Cmd_Array[];

#define CMDINVALID 0xFFFF
#define CMDSHOW 0x068E  // show
#define CMDMOTOR 0x15F4  // motor
#define CMDROTATE 0x2479  // rotate
#define CMDSET 0x0152  // set

#define OPTINVALID 0xFFFF
#define OPTSPEED 0x048E  // -speed
#define OPTLEFT 0x052B  // -left
#define OPTRIGHT 0x1551  // -right
#define OPTFORWARD 0x7ACD  // -forward
#define OPTREVERSE 0x9B71  // -reverse
#define OPTSTOP 0x04FE  // -stop

#define RET_OK 0
#define RET_ERROR 1
#define RET_QUIET 2
#define RET_UNSUPPORTED 3
#define RET_NO_PROMPT 4

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);
int8_t CmdMotor(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);
int8_t CmdRotate(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);
int8_t CmdSet(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);

#endif