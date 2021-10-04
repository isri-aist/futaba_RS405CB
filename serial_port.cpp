#include <iostream>
#include <string.h>
#include "serial_port.h"

//#define VERBOSE

SerialPort::SerialPort(const char *device_name, const int baudrate) : opened(false)
{
	fd = open(device_name, O_RDWR);
	if(fd < 0) {
		std::cerr << "serial port open error. device name: " << device_name << std::endl;
	} else {
		opened = true;

		struct termios tio;
		bzero(&tio, sizeof(tio));
		tio.c_cflag += CREAD; // enable receive
		tio.c_cflag += CLOCAL; // local line (no modem)
		tio.c_cflag += CS8; // data bit: 8bits
		tio.c_cflag += 0; // stop bit: 1bit
		tio.c_cflag += 0; // parity: none

		int baud = B9600;
		if(baudrate == 9600) {
			baud = B9600;
		} else if(baudrate == 14400) {
			//baud = B14400;
			std::cerr << "Error: unsupported baudrate (14400). Using default baudrate (9600)" << std::endl;
		} else if(baudrate == 19200) {
			baud = B19200;
		} else if(baudrate == 28800) {
			//baud = B28800;
			std::cerr << "Error: unsupported baudrate (28800). Using default baudrate (9600)" << std::endl;
		} else if(baudrate == 38400) {
			baud = B38400;
		} else if(baudrate == 54600) {
			//baud = B54600;
			std::cerr << "Error: unsupported baudrate (54600). Using default baudrate (9600)" << std::endl;
		} else if(baudrate == 76800) {
			//baud = B76800
			std::cerr << "Error: unsupported baudrate (76800). Using default baudrate (9600)" << std::endl;
		} else if(baudrate == 115200) {
			baud = B115200;
		} else if(baudrate == 153600) {
			//baud = B153600;
			std::cerr << "Error: unsupported baudrate (153600). Using default baudrate (9600)" << std::endl;
		} else if(baudrate == 230400) {
			baud = B230400;
		} else if(baudrate == 460800) {
			baud = B460800;
		}
		cfsetispeed(&tio, baud);
		cfsetospeed(&tio, baud);

		cfmakeraw(&tio);

		tcsetattr(fd, TCSANOW, &tio);

		ioctl(fd, TCSETS, &tio);
	}
}

SerialPort::~SerialPort()
{
	if(opened) {
		close(fd);
	}
}

int SerialPort::writeData(std::vector<unsigned char> data)
{
#ifdef VERBOSE
        std::cout << "send["  << data.size() << "]:";
	for (unsigned int i=0; i<data.size(); i++){
	        printf("%02x ", data[i]);
	}
	printf("\n");
#endif
	if(opened)
		return write(fd, data.data(), data.size());
	else
		return -1;
}

int SerialPort::readData(std::vector<unsigned char> &data)
{
	if(opened) {
		unsigned char buf[1024];
		const int ret = read(fd, buf, sizeof(buf));
#ifdef VERBOSE
		std::cout << "recv[" << ret << "]:";
		for (int i=0; i<ret; i++){
		        printf("%02x ", buf[i]);
		}
		printf("\n");
#endif
		data = std::vector<unsigned char>(buf, buf + ret);
		return ret;
	} else {
		return -1;
	}
}

