#include "RobotOpenGS.h"

/* Constructor */
ROAnalog::ROAnalog(uint8_t channel)
{
    _channel = channel;
}

int ROAnalog::read() {
	return analogRead(_channel);
}