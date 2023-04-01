#include "tic_stepper_controller.hpp"

namespace ss
{
	TicDriver::TicDriver() : isInitialized(false)
	{
	}

	TicDriver::~TicDriver()
	{
	}

	int TicDriver::init(Serial *port, int initial_position)
	{
		if (setPort(port) != 0)
		{
			return -1;
		}

		isInitialized = true;

		return 0;
	}

	int TicDriver::setPort(Serial *port)
	{
		uint8_t detectByte = 0xAA
		_port = port;
		if (_port == nullptr)
		{
			ROS_ERROR("Serial port pointer supplied is not a valid pointer!");
			return -1;
		}

		if (port->Write(&detectByte, 1) != 1)
		{
			ROS_ERROR("Serial port failed to write baud-detection bit!");
			return -1;
		}

		return 0;
	}
}
