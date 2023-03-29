#include "Arduino.h"
#include "Commands.h"
#include "CommandInterface.h"
#include "ProjectDef.h"
#include "utils.h"
#include "motors.h"
#include <stdio.h>

const struct cmd_str Cmd_Array[] = {
	{CMDSHOW, CmdShow},
	{CMDMOTOR, CmdMotor},
	{CMDROTATE, CmdRotate},
	{CMDSET, CmdSet},
	{OPTINVALID, NULL}
};

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	snprintf(cmdBuf, bufSize, "%s: %s %s v%d.%d.%d", projectClass, projectTeam, projectName, MAJOR_VER, MINOR_VER, BUILD);

	return RET_QUIET;
}

int8_t CmdMotor(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	uint16_t token;
	int8_t retVal = RET_ERROR;

	if (argc < 2)
	{
		return RET_ERROR;
	}

	token = TokenToNum(argv[1]);

	switch (token)
	{
	case OPTLEFT:
		moveLeft();
		retVal = RET_OK;
		break;
	case OPTRIGHT:
		moveRight();
		retVal = RET_OK;
		break;
	case OPTFORWARD:
		moveForward();
		retVal = RET_OK;
		break;
	case OPTREVERSE:
		moveReverse();
		retVal = RET_OK;
		break;
	case OPTSTOP:
		moveStop();
		retVal = RET_OK;
		break;
	}

	return retVal;
}

int8_t CmdRotate(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	float rotation;
	int8_t retVal = RET_ERROR;

	if (argc < 2)
	{
		return RET_ERROR;
	}

	if (!isFloat(argv[1]))
	{
		return RET_ERROR;
	}

	// run rotate
	retVal = RET_UNSUPPORTED;

	return retVal;
}

int8_t CmdSet(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	int8_t retVal = RET_ERROR;
	uint16_t token;

	if (argc < 3)
	{
		return retVal;
	}
	if (!isInteger(argv[2]))
	{
		return retVal;
	}

	token = TokenToNum(argv[1]);

	switch (token)
	{
	case OPTSPEED:
    uint8_t Speed = (uint8_t)atoi(argv[2]);
    setMotorSpeed(Speed);
		retVal = RET_OK;
		break;
	}

	return retVal;
}