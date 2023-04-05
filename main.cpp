#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "RS405CB.h"

int main(int argc, char *argv[])
{
	int id = 0;
	double ang = 0;
	const char *dev = "/dev/ttyUSB0";

	for (int i = 1; i < argc; i++) {
	  if (strcmp(argv[i], "-id") == 0) {
	    id = atoi(argv[++i]);
	  }
	  else if (strcmp(argv[i], "-dev") == 0) {
	    dev = argv[++i];
	  }
	  else if (strcmp(argv[i], "-ang") == 0) {
	    ang = atof(argv[++i]);
	  }
	}
	
	RS405CB servo(dev, 115200);

	const double voltage = servo.getVoltage(id);
	std::cout << "voltage: " << voltage << std::endl;
	const int temperature = servo.getTemperature(id);
	std::cout << "temperature: " << temperature << std::endl;
	const int torqueEnable = servo.getTorqueEnable(id);
	std::cout << "torque enable (before): " << torqueEnable << std::endl;
	const double angle = servo.getAngle(id);
	std::cout << "angle (before): " << angle << std::endl;
	servo.setTorque(id, true); // torque on
	servo.setAngle(id, ang);

	return 0;
}

