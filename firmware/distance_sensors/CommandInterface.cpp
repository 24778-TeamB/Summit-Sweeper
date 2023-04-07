#include "Arduino.h"
#include "CommandInterface.h"
#include "Commands.h"
#include "ProjectDef.h"
#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>

char CmdBuff[CMD_LINE_BUF_SIZE];
uint16_t CbIndex = 0;

const char strCR[] = "\r\n";
const char strOK[] = "\r\nOK\r\n";
const char strUnsupported[] = "\r\nUnsupported\r\n";
const char strError[] = "\r\nERROR\r\n";
const char strPrompt[] = ">";

static uint16_t CmdLineArgParse(char *buf, char **argv)
{
	char *temp;
	uint16_t argc = 0;

	for (temp = buf; argc < NUMTOKENS; argc++)
	{
		assert(isgraph(*temp) || isspace(*temp) || *temp == '\0');
		while (isspace(*temp))
		{
			temp++;
		}
		if (*temp == '\0')
		{
			break;
		}
		argv[argc] = temp;
		while (isgraph(*temp))
		{
			temp++;
		}
	}

	return argc;
}

int8_t ProcessCmdLine(char *cmdBuf, uint16_t bufSize)
{
	int16_t cmdIdx;
	uint16_t argc;
	char *argv[NUMTOKENS];
	int8_t retVal = RET_ERROR;
	uint16_t cmd;

	argc = CmdLineArgParse(cmdBuf, argv);

	if (argc > 0)
	{
		cmd = TokenToNum(argv[0]);

		for (cmdIdx = 0; Cmd_Array[cmdIdx].ftn != NULL; cmdIdx++)
		{
			if (Cmd_Array[cmdIdx].cmd == cmd)
			{
				cmdBuf[0] = '\0';
				retVal = Cmd_Array[cmdIdx].ftn(cmdBuf, bufSize, argc, (const char **)argv);
				break;
			}
		}

		if (Cmd_Array[cmdIdx].ftn == NULL)
		{
			cmdBuf[0] = '\0';
			retVal = RET_ERROR;
		}

		switch (retVal)
		{
		case RET_QUIET:
			strcat(cmdBuf, strCR);
			break;
		case RET_OK:
			strcat(cmdBuf, strOK);
			break;
		case RET_ERROR:
			strcat(cmdBuf, strError);
			break;
		case RET_UNSUPPORTED:
			strcat(cmdBuf, strUnsupported);
			break;
		}
	}

 if (retVal != RET_NO_PROMPT)
 {
  strcat(cmdBuf, strPrompt);
 }

	return retVal;
}

uint16_t TokenToNum(char const *token)
{
	uint16_t i = 0, uiOption = 0;

	if (token[0] == '-')
	{
		token++;
	}

	for (; isalnum(token[i]); i++)
	{
		uiOption += isdigit(token[i]) ? (token[i] - '0') << (i * 2) : (tolower(token[i]) - 'a') << (i * 2);
	}

	return uiOption;
}

static void serialFlush()
{
	while (Serial.available() > 0)
	{
		Serial.read();
	}
}

void CIClear()
{
	serialFlush();
	CbIndex = 0;
	CmdBuff[0] = '\0';
}

uint16_t GetCommand(void)
{
	uint8_t temp_ch;
	uint16_t ret = RX_INCOMPLETE;

	while (Serial.available())
	{
		temp_ch = Serial.read();

		if (temp_ch == CH_BS && CbIndex != 0)
		{
			CbIndex--;
		}
		else if (temp_ch == CH_CR)
		{
			CmdBuff[CbIndex] = '\0';
			ret = RX_COMPLETE;
			break;
		}
		else if (isprint(temp_ch) && (CbIndex < (CMD_LINE_BUF_SIZE - 1)))
		{
			CmdBuff[CbIndex++] = temp_ch;
		}
		else if (CbIndex >= (CMD_LINE_BUF_SIZE - 1))
		{
			ret = RX_OVERFLOW;
			break;
		}
	}

	return ret;
}

void DispatchCommand(void)
{
	ProcessCmdLine(CmdBuff, CMD_LINE_BUF_SIZE);
	Serial.write(CmdBuff);

	CmdBuff[0] = '\0';
	CbIndex = 0;
}
