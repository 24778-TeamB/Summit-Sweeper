#include "CommandInterface.h"
#include "Commands.h"
#include "ProjectDef.h"
#include "motors.h"
#include <stdint.h>

volatile uint32_t gEvents;
uint16_t transfer_status;

void setup(void)
{
    // Configure Serial
    Serial.begin(BAUD);
    CIClear();

    // Configure motors
    setupMotors();
    // setMotorSpeed(255);

    gEvents = E_NO_EVENT;
}

void loop(void)
{
    if (gEvents & E_SERIAL_ACTIVE)
    {
        transfer_status = GetCommand();
        if (transfer_status == RX_COMPLETE)
        {
            DispatchCommand();
        }
        else if (transfer_status >= RX_OVERFLOW)
        {
            Serial.write("Receive Buffer Overflowed!");
            CIClear();
        }
        gEvents &= ~E_SERIAL_ACTIVE;
    }
}

void serialEvent(void)
{
    if (Serial.available())
    {
        gEvents |= E_SERIAL_ACTIVE;
    }
}
