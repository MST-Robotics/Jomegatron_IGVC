#ifndef MIDGPACKET_H
#define MIDGPACKET_H
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

using namespace std;

#include "midgPacketSTATUS.h"
#include "midgPacketAIDVEL.h"
#include "midgPacketNAVPV.h"
#include "midgPacketNAVHDG.h"
#include "midgPacketNAVSENSE.h"
#include "midgPacketGPSPV.h"
#include "midgPacketGPSCLOCK.h"
#include "midgPacketUTCTIME.h"
#include "midgPacketIMUMAG.h"

struct magic_fix_t
{
    bool gotafix;
    double x;
    double y;
    double z;
};


static magic_fix_t startdata={false, 0, 0, 0};

enum MIDG_MESSAGE_TYPE
{
MIDG_MESSAGE_INVALID = 0,
MIDG_MESSAGE_STATUS = 1,
MIDG_MESSAGE_IMUDATA = 2,
MIDG_MESSAGE_IMUMAG = 3,
MIDG_MESSAGE_NAVSENSEDATA = 10,
MIDG_MESSAGE_NAVPVDATA = 12,
MIDG_MESSAGE_NAVHDGDATA = 13,
MIDG_MESSAGE_NAVACCDATA = 15,
MIDG_MESSAGE_GPSPVDATA = 20,
MIDG_MESSAGE_GPSSVI = 21,
MIDG_MESSAGE_GPSRAW = 22,
MIDG_MESSAGE_COUNT = 22,
MIDG_MESSAGE_GPSCLOCK = 23,
MIDG_MESSAGE_GPSEPH = 24,
MIDG_MESSAGE_UTCTIME = 25,
MIDG_MESSAGE_TIMEERROR = 26,
MIDG_MESSAGE_TIMEPULSE = 27,
MIDG_MESSAGE_TIMEMARK = 28
};


static const char * MIDG_MESSAGE_STRING[]=
{
"INVALID MESSAGE",
"Status",
"IMU Data",
"Magnetometer Data",
"",
"",
"",
"",
"",
"",
"Navigation Sense Data",
"",
"Navigation Position/Velocity Data",
"Navigation Heading/Mag Data",
"",
"Navigation Accelerometer Data",
"",
"",
"",
"",
"GPS Position/Velocity Data",
"GPS SVI Data",
"GPS RAW Data",
"GPS Receiver Clock Solution",
"GPS Satellite Ephemeris Data",
"UTC Time",
"Time Error Information",
"Time Pulse Information",
"Time Mark Information"
};

class midgPacket
{
public:
midgPacket( int fd );
midgPacket( const midgPacket & p );
~midgPacket();
bool init( int fd );
bool passesChecksum();
const MIDG_MESSAGE_TYPE getMessageID();

//private:
MIDG_MESSAGE_TYPE messageID;
uint8_t count;
uint8_t * data;
uint8_t cksum0;
uint8_t cksum1;

msg_STATUS handle_msg_STATUS();
msg_NAVSENSE handle_msg_NAVSENSE();
msg_NAVPV handle_msg_NAVPV();
msg_NAVHDG handle_msg_NAVHDG();
msg_GPSPV handle_msg_GPSPV();
msg_GPSCLOCK handle_msg_GPSCLOCK();
msg_UTCTIME handle_msg_UTCTIME();
msg_IMUMAG handle_msg_IMUMAG();
};

midgPacket getonepacket( int fd );

void send_msg_AIDVEL( int fd, msg_AIDVEL input );
double getHeading();

#endif
