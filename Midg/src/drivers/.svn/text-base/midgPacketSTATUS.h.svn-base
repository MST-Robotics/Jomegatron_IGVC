#ifndef MIDGPACKETSTATUS_H
#define MIDGPACKETSTATUS_H
//#include "midgPacket.h"

enum MSG_STATUS_MODE
{
STATUS_MODE_IMU=1,
STATUS_MODE_VGINIT=2,
STATUS_MODE_VGFAST=3,
STATUS_MODE_VGMEDIUM=4,
STATUS_MODE_VGSLOW=5,
STATUS_MODE_VGSE=6,
STATUS_MODE_INS=7
};

struct msg_STATUS
{
//time in seconds, resolution to .01s
double timestamp;

bool timestampisGPStime;
MSG_STATUS_MODE currentmode;
bool nonvolConfigValid;
bool DGPS;

//temp in C, resolution to .01K
double temperature;
};

#endif
