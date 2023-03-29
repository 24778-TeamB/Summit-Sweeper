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
#define CMDDIST 0x0603  // dist

#define OPTINVALID 0xFFFF
#define OPTFRONT 0x1769  // -front
#define OPTLEFT 0x052B  // -left
#define OPTRIGHT 0x1551  // -right
#define OPTDOWN 0x04DB  // -down

#define RET_OK 0
#define RET_ERROR 1
#define RET_QUIET 2
#define RET_UNSUPPORTED 3
#define RET_NO_PROMPT 4

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);
int8_t CmdDist(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv);

#endif