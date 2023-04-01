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

	int TicDriver::set(int32_t position)
	{
		uint32_t value = position;
		uint8_t command[6];

		command[0] = static_cast<uint8_t>(TicCommands::SetTargetPosition);
		command[1] = ((value >> 7) & 1) |
			((value >> 14) & 2) |
			((value >> 21) & 4) |
			((value >> 28) & 8);
		command[2] = value & 0x7F;
		command[3] = (value >> 8) & 0x7F;
		command[4] = (value >> 16) & 0x7F;
		command[5] = (value >> 24) & 0x7F;

		return _port->Write(command, sizeof(command));
	}
}
