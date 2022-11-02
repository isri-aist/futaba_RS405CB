#ifndef RS405CB_H
#define RS405CB_H

#include "rs405cb_types.h"
#include "serial_port.h"
#include <utility>
#include <vector>

enum RS405CB_BAUDRATE
{
  BAUDRATE_9600 = 0x00,
  BAUDRATE_14400 = 0x01,
  BAUDRATE_19200 = 0x02,
  BAUDRATE_28800 = 0x03,
  BAUDRATE_38400 = 0x04,
  BAUDRATE_54600 = 0x05,
  BAUDRATE_76800 = 0x06,
  BAUDRATE_115200 = 0x07,
  BAUDRATE_153600 = 0x08,
  BAUDRATE_230400 = 0x09,
  BAUDRATE_460800 = 0x0A,
};

class RS405CB
{
public:
  RS405CB(const char *, const int);
  ~RS405CB();

  /** Get the motor timeout (in ms)
   *
   * The motor should be considered offline if any read/write operation returns false
   */
  long timeout() const noexcept
  {
    return timeout_ms;
  }

  /** Set the motor timeout (in ms) */
  void timeout(long to) noexcept
  {
    timeout_ms = to;
  }

  /*
   * get the ROM memory map of the servo
   * put result in \p rom
   * return true if succeeds
   * argument: servo id
   */
  [[nodiscard]] bool getDataFromROM(const int id, ROM & rom);

  /*
   * get the RAM memory map of the servo
   * return true if succeeds
   * argument: servo id
   */
  [[nodiscard]] bool getDataFromRAM(const int id, RAM & ram);

  /*
   * get temperature limit that turns of the servo
   * put result in \p temperature in C
   * return true if it worked, false otherwise
   * argument: servo id
   */
  [[nodiscard]] bool getTemperatureLimit(const int id, int & temp_limit);

  /*
   * get current voltage
   * put result in \p voltage
   * return: current voltage. unit is volt [V]
   * argument: servo id
   */
  [[nodiscard]] bool getVoltage(const int id, double & voltage);

  /*
   * get current temperature
   * return: current temperature. unit is degree [C]
   * argument: servo id
   */
  [[nodiscard]] bool getTemperature(const int id, int & temperature);

  /*
   * get current load
   * return: motor current. unit is milli ampere [mA]
   * argument: servo id
   */
  [[nodiscard]] bool getLoad(const int id, int & load);

  /*
   * set motor torque
   * return: written length. if error, return -1
   * arguments: servo id and torque setting
   *   true: torque on
   *   false: torque off
   */
  [[nodiscard]] bool setTorque(const int id, bool torqueOn);

  /*
   * get torque enable
   * return: condition of servo's torque: 0 for torque OFF, 1 for torque ON, 2 for break mode
   * argument: servo id
   */
  [[nodiscard]] bool getTorqueEnable(const int id, bool & isOn);

  /*
   * get current angle
   * return: current angle. unit is degree
   * argument: servo id
   */
  [[nodiscard]] bool getAngle(const int id, double & angle);

  /*
   * set target angle
   * return: written length. if error, return -1
   * arguments: servo id and target angle
   *   valid angle range is -150 <= angle <= 150.
   *   unit is degree.
   */
  [[nodiscard]] bool setAngle(const int id, double target);

  // FIXME We don't use this for now
  /*
   * set target angle to multi servo
   * return: written length. if error, return -1
   * argument: a std::vector consisting of servo id and target angle.
   */
  // int setAngles(std::vector<std::pair<int, double>>);

  /*
   * set moving time
   * return: written length. if error, return -1
   * arguments: servo id and moving time
   *   unit is second [s]
   *   moving time is zero means use max velocity.
   */
  [[nodiscard]] bool setMovingTime(const int id, double movingTime);

  // FIXME Implement later: we don't use this for now
  /*
   * set angle and moving time
   * return: written length. if error, return -1
   * arguments: servo id, target angle and moving time
   */
  /*int setAngleAndMovingTime(const int, double, double);*/

  /*
   * store memory map to flash ROM. need to wait 1 second for writing time.
   * return: written length. if error, return -1
   * argument: servo id
   */
  [[nodiscard]] bool storeDataToROM(const int id);

  /*
   * reboot servo motor.
   * return: written length. if error, return -1
   * argument: servo id
   */
  [[nodiscard]] bool reboot(const int id);

  // FIXME Implement later: we don't use this for now
  /*
   * return: written length. if error, return -1
   * argument: servo id
   */
  // int resetMemoryMap(const int);

  // FIXME Implement later: we don't use this for now
  /*
   * set new ID
   * return: written length, if error, return -1
   * arguments: current servo ID and new servo ID
   */
  // int setServoID(const int, const int);

  // FIXME Implement later: we don't use this for now
  /*
   * set rotate direction
   * return: written length, if error, return -1
   * arguments: servo ID and direction
   *   true: reverse mode
   *   false: no reverse mode
   */
  // int setReverseMode(const int, const bool);

  // FIXME Implement later: we don't use this for now
  /*
   * set baudrate for serial communication
   * return: written length, if error, return -1
   * arguments: servo ID and baudrate
   *   able to use baudrate are: 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 460800.
   *   set baudrate by enum RS405CB_BAUDRATE.
   *   need to reboot after change baudrate
   */
  // int setBaudrate(const int, const RS405CB_BAUDRATE);

  [[nodiscard]] bool setMaxTorque(const int id, const unsigned char max_torque);

  bool isOpen()
  {
    return port.isOpen();
  }

private:
  SerialPort port;
  [[nodiscard]] bool readACK();
  [[nodiscard]] bool sendShortPacket(const int id,
                                     unsigned char flags,
                                     unsigned char address,
                                     unsigned char length,
                                     unsigned char count);
  [[nodiscard]] bool sendAndReceiveShortPacket(const int id,
                                               unsigned char flag,
                                               unsigned char address,
                                               unsigned char length,
                                               unsigned char count);
  //[[nodiscard]] bool sendLongPacket(unsigned char, unsigned char, unsigned char, std::vector<unsigned char>);

  [[nodiscard]] bool receivePacket();

public:
  bool isTemperatureError()
  {
    return flags & 0x80;
  }
  bool isTemperatureAlarm()
  {
    return flags & 0x20;
  }

private:
  std::vector<unsigned char> recv_buffer;
  size_t recv_size;
  std::vector<unsigned char> write_buffer;
  size_t write_size;
  std::vector<unsigned char> write_data;
  std::vector<unsigned char> read_data;
  long timeout_ms = 10;
  unsigned char flags; // ToDo: associate it with id
};

#endif

