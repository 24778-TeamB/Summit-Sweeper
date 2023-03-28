#include "ros/ros.h"
#include <termios.h>
#include <vector>
#include <ostream>
#include <cstdint>

using std::vector;

std::ostream& operator<<(std::ostream &os, vector<uint8_t> v)
{
	for (auto iter = v.begin(); iter < v.end(); iter++)
	{
		os << static_cast<int>(*iter) << ' ';
	}

	return os;
}

uint8_t getKey(void)
{
	return fgetc(stdin);
}

void PrintHelp()
{
	std::cout << "ArrowUp -> forward\n"
		"ArrowDown -> reverse\n"
		"ArrowLeft -> left\n"
		"ArrowRight -> right\n"
		"Press ? to see this message again" << std:endl;
}

int main(int argc, char **argv)
{
	PrintHelp();
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle nh;

	ros::Publisher pubHorizontal = nh.advertise<std_msgs::Int8>("horizontal_control", 1);

	std_msgs::Int8 base_movement = 4;

	struct termios attr_backup;
	tcgetattr(fileno(stdin), &attr_backup);

	struct termios attr_new;
	memcpy(&attr_new, &attr_backup, sizeof(struct termios));
	attr_new.c_lflag &= ~(ECHO | ICANON);
	attr_new.c_cc[VTIME] = 0;
	attr_new.c_cc[VMIN] = 0;

	tcsetattr(fileno(stdin), TCSANOW, &attr_new);

	ros::Rate rate(10);

	while (ros::ok())
	{
		std_msgs::Int8 movement = base_movement;
		uint8_t key;
		vector<uint8_t> keys;

		do
		{
			key = getKey();
			keys.push_back(key);
		} while (255 != key);

		tcflush(fileno(stdin), TCIFLUSH);

		ROS_DEBUG_STREAM(keys);

		switch(keys[0])
		{
			case '?':
				PrintHelp();
				break;
			case 27:
				switch(keys[2])
				{
					case 'A': // Up key
						movement = 2;
						break;
					case 'B': // down key
						movement = 3;
						break;
					case 'C': // right key
						movement = 1;
						break;
					case 'D': // left key
						movement = 0;
						break;
				}
				break;
		}

		pubHorizontal.publish(movement);
		rate.sleep();
	}

	tcsetattr(fileno(stdin), TCSANOW, &attr_backup);

	return 0;
}

