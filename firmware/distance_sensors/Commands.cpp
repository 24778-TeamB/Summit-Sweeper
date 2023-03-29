#include "Arduino.h"
#include "Commands.h"
#include "CommandInterface.h"
#include "ProjectDef.h"
#include "utils.h"
#include "ultrasonic.h"
#include <stdio.h>

const struct cmd_str Cmd_Array[] = {
	{CMDSHOW, CmdShow},
	{CMDDIST, CmdDist},
	{OPTINVALID, NULL}
};

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	snprintf(cmdBuf, bufSize, "%s: %s %s v%d.%d.%d", projectClass, projectTeam, projectName, MAJOR_VER, MINOR_VER, BUILD);

	return RET_QUIET;
}

int8_t CmdDist(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	int8_t retVal = RET_ERROR;
	uint16_t token = OPTINVALID;
	ultrasonic_t data;
	String floatBuffer[3];

	if (argc < 2)
	{
		return retVal;
	}

	token = TokenToNum(argv[1]);

	switch (token)
	{
	case OPTLEFT:
		data = getLeft();
		break;
	case OPTRIGHT:
		data = getRight();
		break;
	case OPTFRONT:
		data = getFront();
		break;
	case OPTDOWN:
		data = getDown();
		break;
	default:
		return retVal;
		break;
	}

	floatBuffer[0] = String(data.headModule, 2);
	floatBuffer[1] = String(data.middleModule, 2);
	floatBuffer[2] = String(data.tailModule, 2);
	snprintf(cmdBuf, bufSize, "%s, %s, %s", floatBuffer[0].c_str(), floatBuffer[1].c_str(), floatBuffer[2].c_str());
	retVal = RET_QUIET;

	return retVal;
}
