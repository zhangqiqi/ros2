#include "SerialPort.h"
#include <cerrno>
#include <cstdio>
#include <stdarg.h>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <cstring>
#include <unistd.h>

extern "C" {
extern int open(const char *file, int oflag, ...);
extern int close(int fd);
extern ssize_t read(int fd, void *buf, size_t count);
extern ssize_t write(int fd, const void *buf, size_t n);

}

using namespace Common;

SerialPort::SerialPort(const std::string &device_name, int baudrate)
	: name(device_name), baudrate(baudrate)
{
	
}


SerialPort::~SerialPort(void)
{
	this->close();
}


void SerialPort::set_errstring(char const *fmt, ...)
{
	char _err_string[512] = {0};
	va_list argp;
	va_start(argp, fmt);
	vsnprintf(_err_string, sizeof(_err_string), fmt, argp);
	va_end(argp);

	errstring = _err_string;
}


const std::string &SerialPort::get_errstring()
{
	return errstring;
}


int32_t SerialPort::set_opt(void)
{
	struct termios new_tio;
	struct termios old_tio;
	
	bzero(&new_tio, sizeof(new_tio));
	bzero(&old_tio, sizeof(old_tio));

	if (0 != tcgetattr(fd, &old_tio))
	{
		set_errstring("%s", strerror(errno));
		return -1;
	}

	switch (baudrate) {
		case 115200:
			cfsetspeed(&new_tio, B115200);
			break;
		case 230400:
			cfsetspeed(&new_tio, B230400);
			break;
		default:
			break;
	}

	new_tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | ICRNL | IXON);
	new_tio.c_oflag &= ~OPOST;
	new_tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	new_tio.c_cflag &= ~(CSIZE | PARENB);
	new_tio.c_cflag |= CS8;	

	new_tio.c_cc[VTIME] = 10;
	new_tio.c_cc[VMIN] = 0;

	tcflush(fd, TCIOFLUSH);	

	if (0 != tcsetattr(fd, TCSANOW, &new_tio))
	{
		set_errstring("%s", strerror(errno));
		return -2;
	}
	return 0;
}


void SerialPort::flush(void)
{
	tcflush(fd, TCIOFLUSH);
}


int32_t SerialPort::open()
{
	fd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		set_errstring("open bau dev port %s, failed, err: %d, %s", name.c_str(), errno, strerror(errno));
		return -1;
	}

	int flags = 0;
	flags = fcntl(fd, F_GETFL, 0);
	flags &= ~O_NONBLOCK;

	if (fcntl(fd, F_SETFL, flags) < 0)
	{
		set_errstring("fcntl bau dev failed, err: %s", strerror(errno));
		return -2;
	}

	return set_opt();
}

int32_t SerialPort::close()
{
	return ::close(fd);
}


int32_t SerialPort::read(uint8_t *buffer, int32_t size)
{
	int32_t ret_size = 0;

	ret_size = ::read(fd, buffer, size);
	if (ret_size < 0)
	{
		set_errstring("%s", strerror(errno));
	}

	return ret_size;
}


int32_t SerialPort::write(uint8_t *buffer, int32_t size)
{
	int32_t ret_size = 0;
	ret_size = ::write(fd, buffer, size);
	if (ret_size < 0)
	{
		set_errstring("%s", strerror(errno));
	}
	return ret_size;
}

