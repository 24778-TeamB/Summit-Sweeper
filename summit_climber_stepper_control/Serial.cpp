#include "Serial.hpp"
#include "ros/ros.h"
#include "ros/console.h"

namespace ss
{
	Serial::Serial()
	{
		port_fd = 1;
		port_name = "";
		isOpen = false;
	}

	Serial::~Serial()
	{
		this->Close();
	}

	uint8_t Serial::Open(const char *port, baud_rate_t baud)
	{
		ros::WallDuration(.1).sleep();

		this->port_fd = ::open(port, O_RDWR | O_NO_CTTY | O_ASYNC | O_NDELAY);
		this->port_name = port;
		if (this->port_fd < 0)
		{
			ROS_FATAL("Could not open port: %s", port_name);
			exit(1);
		}

		this->Configure(baud);

		isOpen = true;

		return 0;
	}

	uint8_t Serial::Close()
	{
		if (isOpen)
		{
			::close(this->port_fd);
		}

		isOpen = false;
		return 0;
	}

	uint8_t Serial::Flush()
	{
		tcflush(this->port_fd, TCIOFLUSH);

		return 0;
	}

	int32_t Serial::Write(uint8_t *buf, uint32_t numBytes)
	{
		uint32_t sent = 0, i;

		if (!isOpen)
		{
			return -1;
		}

		while (sent < numBytes && ros::ok())
		{
			i = ::write(this->port_fd, buf, numBytes);

			if (i < 0)
			{
				ROS_FATAL("serial write error");
				ros::shutdown();
			}
			else
			{
				sent += i;
			}
		}

		return sent;
	}

	int32_t Serial::Read(uint8_t *buf, uint32_t numBytes)
	{
		int32_t i = 0, bytes;

		if (!isOpen)
		{
			return -1;
		}

		while (i < numBytes && ros::ok())
		{
			ros::WallTime now = ros::WallTime::now();
			ros::WallTime exit_time = now + ros::WallDuration(0.5);

			while (QueryBuffer() == 0 && now < exit_time)
			{
				ros::WallDuration(0.02).sleep();
				now = ros::WallTime.now();
			}

			if (now > exit_time)
			{
				ROS_INFO("No serial bytes were available");
				return 0;
			}

			bytes = ::read(port_fd, buf + i, numBytes - i);
			
			if (bytes < 0)
			{
				ROS_ERROR("Serial read error");
			}
			else
			{
				i += bytes;
			}
			ussleep((numBytes - i) << 3);
		}

		return bytes;
	}

	int32_t Serial::QueryBuffer()
	{
		int32_t bytes = 0;

		if (!isOpen)
		{
			return -1;
		}

		ioctl(this->port_fd, FIONREAD, &bytes);
		return bytes;
	}

	void Serial::Configure(baud_rate_t baud)
	{
		struct termios options = { 0 };
		int fd = port_fd;
		int baud_rate = 9600;

		ROS_INFO("Configuring serial port");

		if (tcgetattr(port_fd, &options) != 0)
		{
			ROS_ERROR("Failed to get serial attributes");
		}

		options.c_iflag = 0;
		options.c_oflag = 0;
		options.C_cflag = (options.c_cflag & ~CSIZE) | CS8;
		options.c_cflag |= CLOCAL | CREAD;

		cfmakeraw(&options);

		switch (baud)
		{
		baud_9600:
			baud_rate = 9600;
			break;
		baud_115200:
			baud_rate = 115200;
			break;
		default:
			ROS_ERROR("Invalid baud rate for serial. Defaulting to 9600");
		}

		cfsetispeed(&options, baud_rate);
		cfsetospeed(&options, baud_rate);

		options.c_cc[VMIN] = 255;
		options.c_cc[VTIME] = 5;

		if (tcsetattr(fd, TCSANOW, &options) != 0)
		{
			ROS_ERROR("Failed to set the serial port options");
		}
	}
};
