#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <termios.h>
#include <vector>
#include <ostream>
#include <cstdint>
#include <cctype>

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
	char key;
	int count = read(fileno(stdin), &key, 1);
	return count != 0 ? key : 255;
}

void PrintHelp()
{
	std::cout << "W -> forward\n"
		"S -> reverse\n"
		"A -> left\n"
		"D -> right\n"
		"Press ? to see this message again" << std::endl;
}

int main(int argc, char **argv)
{
	PrintHelp();
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle nh;

	ros::Publisher pubHorizontal = nh.advertise<std_msgs::Int8>("horizontal_control", 1);

	std_msgs::Int8 base_movement;
	base_movement.data = 1;

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

		printf("%d\n", key);

		tcflush(fileno(stdin), TCIFLUSH);

		switch(toupper(key))
		{
			case '?':
				PrintHelp();
				break;
			case 'W':
				movement.data = 3;
				break;
			case 'S':
				movement.data = 4;
				break;
			case 'A':
				movement.data = 1;
				break;
			case 'D':
				movement.data = 2;
				break;
		}

		pubHorizontal.publish(movement);
		rate.sleep();
	}

	tcsetattr(fileno(stdin), TCSANOW, &attr_backup);

	return 0;
}

