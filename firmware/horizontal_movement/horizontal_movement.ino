#include "motors.h"
#include "ProjectDef.h"
#include <stdint.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

ros::NodeHandle nh;

void motorsCallback(const std_msgs::UInt8MultiArray & motorSpeeds)
{
	uint8_t speed[NUM_MOTORS];
	uint8_t direction;

	if (motorSpeeds.data_length != (NUM_MOTORS + 1))
	{
		return;
	}

	direction = motorSpeeds.data[0];

	for (uint16_t i = 1; i < (NUM_MOTORS + 1); i++)
	{
    nh.loginfo(String(motorSpeeds.data[i]).c_str());
		speed[i - 1] = motorSpeeds.data[i];
	}

	updateMotors(direction, speed);

 nh.logdebug("Finished speed updates");
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("horizontal_control", &motorsCallback);


void setup(void)
{
	nh.initNode();
	nh.subscribe(sub);
}

void loop(void)
{
	nh.spinOnce();
	delay(1);
}
