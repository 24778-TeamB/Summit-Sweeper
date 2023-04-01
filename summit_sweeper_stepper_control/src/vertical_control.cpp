#include <ros/ros.h>
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
	ros::Publisher pub1 = n.publish("vertical_position1", /*Something else*/);
	ros::Publisher pub2 = n.publish("vertical_position2", /*Something else*/);

	// get serial ports
	port1.Open("somePort", baud_9600);
	port2.Open("somePort", baud_9600);

	ticController1.init(&port1, 0);
	ticController2.init(&port2, 0);

	while (!ros::is_shutdown())
	{
		// publish current positions...
	}

	resetPositions();

	return 0;
}
