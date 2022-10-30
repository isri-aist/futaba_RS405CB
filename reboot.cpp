#include <iostream>
#include <string.h>
#include "RS405CB.h"

int main(int argc, char *argv[])
{
  int sid = 1;
  const char *dev = "/dev/ttyUSB0";

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-dev") == 0) {
      dev = argv[++i];
    }
  }

  RS405CB servo(dev, 115200);

  if (!servo.isOpen()){
    std::cerr << "failed to open " << dev << std::endl;
    return 1;
  }

  std::cout << "About to reboot servo" << std::endl;

  servo.reboot(sid);
  
  std::cout << "Rebooting servo at " << dev << std::endl;

  return 0;
}
