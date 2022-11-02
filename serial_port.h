#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <vector>

class SerialPort
{
public:
  SerialPort(const char * device_name, const int baudrate);
  ~SerialPort();
  int writeData(std::vector<unsigned char>);
  int readData(std::vector<unsigned char> &);
  bool isOpen()
  {
    return opened;
  }

private:
  int fd;
  bool opened;
};

#endif

