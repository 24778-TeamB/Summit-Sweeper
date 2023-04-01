#include <stdint.h>
#include <TimerOne.h>
#include "ProjectDef.h"
#include "Commands.h"
#include "CommandInterface.h"
#include "ultrasonic.h"

volatile uint32_t gEvents;
uint16_t transfer_status;

void setup(void)
{
  // Configure Serial
  Serial.begin(BAUD);
  CIClear();
  
  // Configure sensors
  init_ultrasonic();

  Timer1.initialize(10000000);
  Timer1.attachInterrupt(timerInterrupt);

  gEvents = E_NO_EVENT;
}

void loop(void)
{
  if (gEvents & E_TIMER1)
  {
    updateHead();
    updateMiddle();
    updateTail();
    gEvents &= ~E_TIMER1;
  }
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

void timerInterrupt(void)
{
  gEvents |= E_TIMER1;
}
