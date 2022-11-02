#include "serial_port.h"

#include <cstring>
#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

//#define VERBOSE

SerialPort::SerialPort(const char * device_name, const int baudrate) : opened(false)
{
  fd = open(device_name, O_RDWR | O_NONBLOCK);
  if(fd < 0)
  {
    std::cerr << "serial port open error. device name: " << device_name << std::endl;
  }
  else
  {
    opened = true;

    struct termios tio;
    bzero(&tio, sizeof(tio));
    tio.c_cflag += CREAD; // enable receive
    tio.c_cflag += CLOCAL; // local line (no modem)
    tio.c_cflag += CS8; // data bit: 8bits
    tio.c_cflag += 0; // stop bit: 1bit
    tio.c_cflag += 0; // parity: none

    unsigned int baud = B9600;
    if(baudrate == 9600)
    {
      baud = B9600;
    }
    else if(baudrate == 14400)
    {
      // baud = B14400;
      std::cerr << "Error: unsupported baudrate (14400). Using default baudrate (9600)" << std::endl;
    }
    else if(baudrate == 19200)
    {
      baud = B19200;
    }
    else if(baudrate == 28800)
    {
      // baud = B28800;
      std::cerr << "Error: unsupported baudrate (28800). Using default baudrate (9600)" << std::endl;
    }
    else if(baudrate == 38400)
    {
      baud = B38400;
    }
    else if(baudrate == 54600)
    {
      // baud = B54600;
      std::cerr << "Error: unsupported baudrate (54600). Using default baudrate (9600)" << std::endl;
    }
    else if(baudrate == 76800)
    {
      // baud = B76800
      std::cerr << "Error: unsupported baudrate (76800). Using default baudrate (9600)" << std::endl;
    }
    else if(baudrate == 115200)
    {
      baud = B115200;
    }
    else if(baudrate == 153600)
    {
      // baud = B153600;
      std::cerr << "Error: unsupported baudrate (153600). Using default baudrate (9600)" << std::endl;
    }
    else if(baudrate == 230400)
    {
      baud = B230400;
    }
    else if(baudrate == 460800)
    {
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
  if(opened)
  {
    close(fd);
  }
}

bool SerialPort::writeData(const std::vector<unsigned char> & data, size_t & written, long timeout_ms)
{
  if(!opened)
  {
    return false;
  }
  bool error = false;
  auto handle_write = [&]() {
    ssize_t write_result = write(fd, data.data(), data.size());
    if(write_result <= 0)
    {
      if(errno == EAGAIN)
      {
        return false;
      }
      error = true;
      perror("write");
      return true;
    }
    written = static_cast<size_t>(write_result);
    return true;
  };
  struct timeval start_tv;
  gettimeofday(&start_tv, nullptr);
  struct timeval now_tv;
  while(!handle_write())
  {
    gettimeofday(&now_tv, nullptr);
    long elapsed_ms = (now_tv.tv_sec - start_tv.tv_sec) * 1000 + (now_tv.tv_usec - start_tv.tv_usec) / 1000;
    if(elapsed_ms > timeout_ms)
    {
      std::cerr << "write timeout\n";
      return false;
    }
  }
  return !error;
}

bool SerialPort::readData(std::vector<unsigned char> & data, size_t & readOut, long timeout_ms)
{
  if(!opened)
  {
    return false;
  }
  if(data.size() < 1024)
  {
    data.resize(1024);
  }
  bool error = false;
  auto try_read = [&]() {
    ssize_t read_result = read(fd, data.data(), data.size());
    if(read_result <= 0)
    {
      if(errno == EAGAIN)
      {
        return false;
      }
      error = true;
      perror("read");
      return true;
    }
    readOut = static_cast<size_t>(read_result);
    return true;
  };
  struct timeval start_tv;
  gettimeofday(&start_tv, nullptr);
  struct timeval now_tv;
  while(!try_read())
  {
    gettimeofday(&now_tv, nullptr);
    long elapsed_ms = (now_tv.tv_sec - start_tv.tv_sec) * 1000 + (now_tv.tv_usec - start_tv.tv_usec) / 1000;
    if(elapsed_ms > timeout_ms)
    {
      std::cerr << "read timeout\n";
      return false;
    }
  }
  return !error;
}

