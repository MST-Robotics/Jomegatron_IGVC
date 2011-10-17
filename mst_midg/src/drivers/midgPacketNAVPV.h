#ifndef MIDGPACKETNAVPV_H
#define MIDGPACKETNAVPV_H
//#include "midgPacket.h"

enum MSG_NAVPV_VELFMT
{
VELFMT_ECEF,
VELFMT_ENU,
VELFMT_COUNT
};

static const char * MSG_NAVPV_VELFMT_STRING[]=
{
"ECEF",
"ENU"
};

enum MSG_NAVPV_POSFMT
{
POSFMT_ECEF,
POSFMT_ENU,
POSFMT_LLA1,
POSFMT_LLA2,
POSFMT_COUNT
};

static const char * MSG_NAVPV_POSFMT_STRING[]=
{
"ECEF",
"ENU",
"LLA",
"LLA"
};

struct msg_NAVPV
{
//time in seconds, resolution to .01s
double timestamp;

//position in magic, to be determined units
double xPos;
double yPos;
double zPos;

//velocity in m/s, to be determined axis
double xVel;
double yVel;
double zVel;

bool positionvalid;
bool velocityvalid;

bool timestampisGPStime;
bool DGPS;
MSG_NAVPV_VELFMT VelocityFormat;
MSG_NAVPV_POSFMT PositionFormat;
bool ENUPositionRelativeToFirstFix;
};

#endif
