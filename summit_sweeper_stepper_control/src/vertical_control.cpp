#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "Serial.hpp"
#include "tic_stepper_controller.hpp"
#include <string>
#include <map>
#include <vector>
#include <mutex>

using namespace ss;
Serial port1, port2;
TicDriver ticController1;
TicDriver ticController2;

std::mutex m1;
std::mutex m2;

void setStepperPosition1(/* Something that goes here */)
{
	m1.lock();
	ticController1.setPosition(/* Some data that goes here */); 
	m1.unlock();
}

void setStepperPosition2(/* Something that goes here */)
{
	m2.lock();
	ticController2.setPosition(/* Some data that goes here */);
	m2.unlock();
}

void resetPositions(void)
{
	ticController1.setPosition(0);
	ticController2.setPosition(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vertical_control");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("vertical_control1", 1, setStepperPosition1);
	ros::Subscriber sub2 = n.subscribe("vertical_control2", 1, setStepperPosition2);
	ros::Publisher pub1 = n.advertise<std_msgs::Int32>("vertical_position1", 4);
	ros::Publisher pub2 = n.advertise<std_msgs::Int32>("vertical_position2", 4);

	// get serial ports
	port1.Open("somePort", baud_9600);
	port2.Open("somePort", baud_9600);

	ticController1.init(&port1, 0);
	ticController2.init(&port2, 0);

	ros::Rate rate(10);

	while (ros::ok())
	{
		std_msgs::Int32 pos1_msg, pos2_msg;
		int32_t pos1, pos2;

		m1.lock();
		ticController1.getPosition(&pos1);
		m1.unlock();

		m2.lock();
		ticController2.getPosition(&pos2);
		m2.unlock();

		pos1_msg.data = pos1;
		pos2_msg.data = pos2;

		pub1.publish(pos1_msg);
		pub2.publish(pos2_msg);

		rate.sleep();
	}

	resetPositions();

	return 0;
}
