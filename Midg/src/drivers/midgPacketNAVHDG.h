#ifndef MIDGPACKETNAVHDG_H
#define MIDGPACKETNAVHDG_H
//#include "midgPacket.h"

struct msg_NAVHDG
{
//time in seconds, resolution to .01s
double timestamp;

//magnetomoter measurements(degrees)
double magHeading;
double magDeclination;
double magDip;

//filtered headings stuff (don't understand)
double cog;//Course Over Ground
double sog;//Speed Over Ground
double vup;//Vertical Velocity

bool declinationvalid;
bool dipvalid;
bool timestampisGPStime;
};

#endif
