#include "RS405CB.h"
#include <algorithm>
#include <iostream>
#include <memory.h>
#include <unistd.h>

RS405CB::RS405CB(const char * serial_port_device_name, const int baudrate) : port(serial_port_device_name, baudrate)
{
  recv_buffer.resize(1024);
  write_buffer.resize(1024);
}

RS405CB::~RS405CB() {}

bool RS405CB::sendAndReceiveShortPacket(const int id,
                                        unsigned char flag,
                                        unsigned char address,
                                        unsigned char length,
                                        unsigned char count)
{
  if(!sendShortPacket(id, flag, address, length, count))
  {
    return false;
  }
  return receivePacket();
}

bool RS405CB::readACK()
{
  if(!port.readData(recv_buffer, recv_size, timeout_ms))
  {
    std::cerr << "Failed to read ACK\n";
    return false;
  }
  return true;
}

bool RS405CB::sendShortPacket(const int id,
                              unsigned char flag,
                              unsigned char address,
                              unsigned char length,
                              unsigned char count)
{
  if(length != write_data.size())
  {
    std::cerr << "Coherence issue, length is " << length << " but write_data is " << write_data.size() << "\n";
  }
  size_t header_size = 7;
  size_t tail_size = 1;
  size_t write_expected = (write_data.size() + header_size + tail_size);
  write_buffer.resize(write_expected);

  write_buffer[0] = 0xFA;
  write_buffer[1] = 0xAF;
  write_buffer[2] = static_cast<unsigned char>(id);
  unsigned char check_sum = write_buffer[2];

  size_t write_idx = 3;
  auto write_to_buffer = [&](unsigned char value) {
    write_buffer[write_idx] = value;
    check_sum = check_sum ^ value;
    write_idx++;
  };
  write_to_buffer(flag);
  write_to_buffer(address);
  write_to_buffer(length);
  write_to_buffer(count);

  for(const auto & d : write_data)
  {
    write_to_buffer(d);
  }

  write_buffer[write_buffer.size() - 1] = check_sum;

  bool success = port.writeData(write_buffer, write_size, timeout_ms);
  if(success && write_size != write_expected)
  {
    std::cerr << "Wrote less data (" << write_size << ") than expected (" << write_expected << ")\n";
    return false;
  }
  return success;
}

// int RS405CB::sendLongPacket(unsigned char address,
//                            unsigned char length,
//                            unsigned char count,
//                            std::vector<unsigned char> data)
//{
//  std::vector<unsigned char> buf;
//
//  buf.push_back(0xFA); // header
//  buf.push_back(0xAF); // header
//  buf.push_back(0x00); // id
//  buf.push_back(0x00); // flags
//  buf.push_back(address);
//  buf.push_back(length);
//  buf.push_back(count);
//
//  for(auto d : data)
//  {
//    buf.push_back(d);
//  }
//
//  unsigned char check_sum = buf[2];
//  for(std::size_t i = 3; i < buf.size(); i++)
//  {
//    check_sum = check_sum ^ buf[i];
//  }
//  buf.push_back(check_sum);
//
//  const int write_len = port.writeData(buf);
//  return write_len;
//}

bool RS405CB::receivePacket()
{
  if(!port.readData(recv_buffer, recv_size, timeout_ms))
  {
    return false;
  }
  if(recv_size < 6)
  {
    std::cerr << "Didn't receive enough data (" << recv_size << ") expected at least 6\n";
    return false;
  }
  if(recv_buffer[0] != 0xFD || recv_buffer[1] != 0xDF)
  {
    std::cout << "Received " << recv_size << " but received the wrong header\n";
    fprintf(stderr, "Got %X and %X\n", recv_buffer[0], recv_buffer[1]);
    return false;
  }
  flags = recv_buffer[3];
  if(flags != 0)
  {
    if(flags & 0x80)
    {
      std::cerr << "error. temperature limit\n";
      return false;
    }
    if(flags & 0x20)
    {
      std::cerr << "warning. temperature alarm\n";
    }
    if(flags & 0x08)
    {
      std::cerr << "error. flash writing error\n";
      return false;
    }
  }
  unsigned char data_length = recv_buffer[5];
  if(recv_size < 7 + data_length)
  {
    std::cerr << "Received buffer is smaller than expected (data_length: " << static_cast<size_t>(data_length)
              << ", received: " << recv_size << ")\n";
    return false;
  }
  read_data.resize(data_length);
  for(size_t i = 0; i < data_length; ++i)
  {
    read_data[i] = recv_buffer[7 + i];
  }
  return true;
}

bool RS405CB::getDataFromROM(const int id, ROM & rom)
{
  write_data.resize(0);
  // get data from 0 (0x00) to 29 (0x1D) of memory map; that is, the ROM
  if(!sendAndReceiveShortPacket(id, 0x03, 0x00, 0x00, 0x01))
  {
    return false;
  }
  memcpy(rom.BYTE, read_data.data(), 30 * sizeof(unsigned char));
  return true;
}

bool RS405CB::getDataFromRAM(const int id, RAM & ram)
{
  write_data.resize(0);
  // get data from 30 (0x1E) to 59 (0x3B) of memory map; that is, the RAM
  if(!sendAndReceiveShortPacket(id, 0x05, 0x00, 0x00, 0x01))
  {
    return false;
  }
  memcpy(ram.BYTE, read_data.data(), 30 * sizeof(unsigned char));
  return true;
}

bool RS405CB::getTemperatureLimit(const int id, int & out)
{
  write_data.resize(0);
  // get number from 0 to 29 of memory map
  if(!sendAndReceiveShortPacket(id, 0x03, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = (read_data[15] << 8) | read_data[14];
  return true;
}

bool RS405CB::getVoltage(const int id, double & out)
{
  write_data.resize(0);
  // get number from 42 to 59 of memory map
  if(!sendAndReceiveShortPacket(id, 0x09, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = ((read_data[11] << 8) | read_data[10]) / 100.0;
  return true;
}

bool RS405CB::getTemperature(const int id, int & out)
{
  write_data.resize(0);
  // get number from 42 to 59 of memory map
  if(!sendAndReceiveShortPacket(id, 0x09, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = (read_data[9] << 8) | read_data[8];
  return true;
}

bool RS405CB::getLoad(const int id, int & out)
{
  write_data.resize(0);
  // get number from 42 to 59 of memory map
  if(!sendAndReceiveShortPacket(id, 0x09, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = (read_data[7] << 8) | read_data[6];
  return true;
}

bool RS405CB::getAngle(const int id, double & out)
{
  write_data.resize(0);
  // get number from 42 to 59 of memory map
  if(!sendAndReceiveShortPacket(id, 0x09, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = ((short)((read_data[1] << 8) | read_data[0])) / 10.0;
  return true;
}

bool RS405CB::setTorque(const int id, bool torque_on)
{
  write_data.resize(1);
  write_data[0] = torque_on ? 0x01 : 0x00;
  if(!sendShortPacket(id, 0x01, 0x24, 0x01, 0x01))
  {
    return false;
  }
  return readACK();
}

bool RS405CB::getTorqueEnable(const int id, bool & out)
{
  // get number from 30 to 41 of memory map
  write_data.resize(0);
  if(!sendAndReceiveShortPacket(id, 0x0B, 0x00, 0x00, 0x01))
  {
    return false;
  }
  out = read_data[6] != 0x00;
  return true;
}

bool RS405CB::setAngle(const int id, double angle)
{
  angle = std::max<double>(angle, -150.0);
  angle = std::min<double>(angle, 150.0);
  angle *= 10.0;
  short angle_int = static_cast<signed short>(angle);
  write_data.resize(2);
  write_data[0] = angle_int & 0xff;
  write_data[1] = (angle_int >> 8) & 0xff;
  if(!sendShortPacket(id, 0x01, 0x1e, 0x02, 0x01))
  {
    return false;
  }
  return readACK();
}

// int RS405CB::setAngles(std::vector<std::pair<int, double>> angles)
//{
//  std::vector<unsigned char> data;
//
//  for(auto id_and_angle : angles)
//  {
//    const unsigned char id = id_and_angle.first;
//    double angle = id_and_angle.second;
//    angle = std::max<double>(angle, -150.0);
//    angle = std::min<double>(angle, 150.0);
//    angle *= 10.0;
//    short angle_int = static_cast<signed short>(angle);
//    data.push_back(id);
//    data.push_back(angle_int & 0xff);
//    data.push_back(angle_int >> 8);
//  }
//  const unsigned char length = 3;
//  return sendLongPacket(0x1e, length, angles.size(), data);
//}

bool RS405CB::setMovingTime(const int id, double time)
{
  time *= 100.0;
  unsigned short time_int = static_cast<unsigned short>(time);
  write_data.resize(2);
  write_data[0] = time_int & 0xff;
  write_data[1] = (time_int >> 8) & 0xff;
  return sendShortPacket(id, 0x00, 0x20, 0x02, 0x01);
}

// bool RS405CB::setAngleAndMovingTime(const int id, double angle, double time)
//{
//  std::vector<unsigned char> data;
//
//  angle = std::max<double>(angle, -150.0);
//  angle = std::min<double>(angle, 150.0);
//  angle *= 10.0;
//  short angle_int = static_cast<signed short>(angle);
//  data.push_back(angle_int & 0xff);
//  data.push_back(angle_int >> 8);
//
//  time *= 100.0;
//  unsigned short time_int = static_cast<unsigned short>(time);
//  data.push_back(time_int & 0xff);
//  data.push_back(time_int >> 8);
//  sendShortPacket(id, 0x00, 0x1e, 0x04, 0x01, data);
//
//  return 0;
//}

bool RS405CB::storeDataToROM(const int id)
{
  write_data.resize(0);
  bool return_value = sendShortPacket(id, 0x40, 0xff, 0x00, 0x00);
  sleep(1);
  return return_value;
}

bool RS405CB::reboot(const int id)
{
  write_data.resize(0);
  bool out = sendShortPacket(id, 0x20, 0xff, 0x00, 0x00);
  sleep(1);
  return out;
}

// int RS405CB::resetMemoryMap(const int id)
//{
//  // reset memory map to factory setting
//  return sendShortPacket(id, 0x10, 0xff, 0xff, 0x00);
//}
//
// int RS405CB::setServoID(const int current_id, const int new_id)
//{
//  std::vector<unsigned char> data(1, static_cast<unsigned char>(new_id));
//
//  return sendShortPacket(current_id, 0x00, 0x04, 0x01, 0x01, data);
//}
//
// int RS405CB::setReverseMode(const int id, const bool reverse)
//{
//  std::vector<unsigned char> data;
//
//  if(reverse)
//  {
//    data.push_back(0x01);
//  }
//  else
//  {
//    data.push_back(0x00);
//  }
//
//  return sendShortPacket(id, 0x00, 0x04, 0x01, 0x01, data);
//}
//
// int RS405CB::setBaudrate(const int id, const RS405CB_BAUDRATE baudrate)
//{
//  std::vector<unsigned char> data(1, baudrate);
//
//  return sendShortPacket(id, 0x00, 0x06, 0x01, 0x01, data);
//}

bool RS405CB::setMaxTorque(const int id, const unsigned char max_torque)
{
  write_data.resize(1);
  write_data[0] = max_torque;
  if(!sendShortPacket(id, 0x01, 0x23, 0x01, 0x01))
  {
    return false;
  }
  return readACK();
}

