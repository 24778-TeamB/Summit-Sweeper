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

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerInterrupt);

  gEvents = E_NO_EVENT;
}

void loop(void)
{
  if (gEvents & E_TIMER1)
  {
    ultrasonic_t left, right, front, down;
    // Serial.println("1");
    updateHead();
    updateMiddle();
    updateTail();
    // Serial.println("2");

    left = getLeft();
    right = getRight();
    front = getFront();
    down = getDown();
    // Serial.println("3");

    // Serial.println(String(left.headModule, 2) + String(",") + String(left.middleModule, 2) + String(",") + String(left.tailModule, 2) + String(",") + String(right.headModule, 2) + String(",") + String(right.middleModule, 2) + String(",") + String(right.tailModule, 2) + String(",") + String(front.headModule, 2) + String(",") + String(front.middleModule, 2) + String(",") + String(front.tailModule, 2) + String(",") + String(down.headModule, 2) + String(",") + String(down.middleModule, 2) + String(",") + String(down.tailModule, 2));
    
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
