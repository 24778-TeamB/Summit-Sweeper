#include "Arduino.h"
#include "Commands.h"
#include "CommandInterface.h"
#include "ProjectDef.h"
#include "sensors.h"
#include <stdio.h>

const struct cmd_str Cmd_Array[] = {
	{CMDSHOW, CmdShow},
	{OPTINVALID, NULL}
};

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc, char const *const *argv)
{
	snprintf(cmdBuf, bufSize, "%s: %s %s v%d.%d.%d", projectClass, projectTeam, projectName, MAJOR_VER, MINOR_VER, BUILD);

	return RET_QUIET;
}