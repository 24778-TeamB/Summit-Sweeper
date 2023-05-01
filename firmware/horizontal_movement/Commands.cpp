#include "Commands.h"
#include "Arduino.h"
#include "CommandInterface.h"
#include "ProjectDef.h"
#include "motors.h"
#include "utils.h"
#include <stdio.h>

const struct cmd_str Cmd_Array[] = {
    { CMDSHOW, CmdShow },     //
    { CMDMOTOR, CmdMotor },   //
    { CMDROTATE, CmdRotate }, //
    { CMDSET, CmdSet },       //
    { CMDPULSE, CmdPulse },   //
    { OPTINVALID, NULL },
};

int8_t CmdShow(char *cmdBuf, uint16_t bufSize, uint16_t argc,
               char const *const *argv)
{
    snprintf(cmdBuf, bufSize, "%s: %s %s v%d.%d.%d", projectClass, projectTeam,
             projectName, MAJOR_VER, MINOR_VER, BUILD);

    return RET_QUIET;
}

int8_t CmdMotor(char *cmdBuf, uint16_t bufSize, uint16_t argc,
                char const *const *argv)
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

int8_t CmdRotate(char *cmdBuf, uint16_t bufSize, uint16_t argc,
                 char const *const *argv)
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

int8_t CmdSet(char *cmdBuf, uint16_t bufSize, uint16_t argc,
              char const *const *argv)
{
    int8_t retVal = RET_ERROR;
    uint16_t token;

    if (argc < 4)
    {
        return retVal;
    }

    token = TokenToNum(argv[1]);

    switch (token)
    {
    case OPTSPEED:
        uint8_t motor, Speed;
        bool success;
        if (!isInteger(argv[3]))
        {
            return retVal;
        }
        if (tolower(argv[2][0]) != 'r' && tolower(argv[2][0]) != 'l')
        {
            return retVal;
        }
        motor = (tolower(argv[2][0]) == 'l') ? 0x10 : 0x00;
        motor |= (uint8_t)(argv[2][1] - '1');

        Speed = (uint8_t)atoi(argv[3]);
        success = setMotorSpeed(motor, Speed);
        retVal = success ? RET_OK : RET_ERROR;
        break;
    }

    return retVal;
}

int8_t CmdPulse(char *cmdBuf, uint16_t bufSize, uint16_t argc,
                char const *const *argv)
{
    int8_t retVal = RET_ERROR;
    uint16_t token;
    uint32_t time;

    if (argc < 3)
    {
        return retVal;
    }

    token = TokenToNum(argv[1]);

    if (!isInteger(argv[2]))
    {
        return retVal;
    }

    time = (uint32_t)atoi(argv[2]);

    switch (token)
    {
    case OPTFORWARD:
        pulseMotors(time, moveForward);
        break;
    case OPTLEFT:
        pulseMotors(time, moveLeft);
        break;
    case OPTRIGHT:
        pulseMotors(time, moveRight);
        break;
    case OPTREVERSE:
        pulseMotors(time, moveReverse);
        break;
    }

    return retVal;
}
