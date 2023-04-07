#ifndef __COMMON_INCLUDE_SERIALPORT_H__
#define __COMMON_INCLUDE_SERIALPORT_H__

#include <string>


namespace Common {

class SerialPort
{
public:
	SerialPort(const std::string &device_name, int baudrate);
	~SerialPort(void);

	int32_t open(void);
	int32_t close(void);
	int32_t read(uint8_t *buffer, int32_t size);
	int32_t write(uint8_t *buffer, int32_t size);
	void flush(void);

	int32_t set_opt(void);

	const std::string &get_errstring(void);

private:

	void set_errstring(char const *fmt, ...);

	const std::string &name;	
	int baudrate;
	int fd;

	std::string errstring;
};


} // namespace Common

#endif // __COMMON_INCLUDE_SERIALPORT_H__

