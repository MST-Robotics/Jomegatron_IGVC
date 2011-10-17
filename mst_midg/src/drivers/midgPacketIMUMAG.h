#ifndef MIDGPACKETIMUMAG_H
#define MIDGPACKETIMUMAG_H
//#include "midgPacket.h"

struct msg_IMUMAG
    {
	double timestamp;
	double mag_x;
	double mag_y;
	double mag_z;
	double mag_y_sine;
	double mag_x_sine;
	double mag_z_sine;
	uint8_t flags;
    };
#endif
