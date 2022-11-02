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
  /** Write data from \p data onto the serial port
   *
   * \p timeout_ms Time after which we consider the link has been broken in ms
   *
   * Returns true if the write worked and put the number of written byte in \p written */
  [[nodiscard]] bool writeData(const std::vector<unsigned char> & data, size_t & written, long timeout_ms);
  /** Read data from the serial port into the provided buffer
   *
   * \p timeout_ms Time after which we consider the link has been broken in ms
   *
   * Returns true if the read succeeded and put the number of written byte in \p read
   */
  [[nodiscard]] bool readData(std::vector<unsigned char> & buffer, size_t & read, long timeout_msg);

  inline bool isOpen() const noexcept
  {
    return opened;
  }

private:
  int fd;
  bool opened;
};

#endif

