#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <string>
#include <memory>

namespace ss
{
	typedef enum {
		baud_9600,
		baud_115200
	} baud_rate_t;

	class Serial
	{
	public:
		Serial();
		~Serial();
		uint8_t Open(const char *port_name, baud_rate_t baud);
		uint8_t Close();
		int32_t Write(uint8_t *buf, uint32_t numBytes);
		int32_t Read(uint8_t *buf, uint32_t numBytes);
		int32_t QueryBuffer();
		uint8_t Flush();
	private:
		void Configure(baud_rate_t baud);

		int port_fd;
		std::string port_name;
		bool isOpen;
	};
};

#endif // !__SERIAL_HPP__

