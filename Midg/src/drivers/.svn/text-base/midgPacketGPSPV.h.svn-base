#ifndef MIDGPACKETGPSPV_H
#define MIDGPACKETGPSPV_H
//#include "midgPacket.h"

enum MSG_GPSPV_VELFMT
{
GPSVELFMT_ECEF,
GPSVELFMT_ENU,
GPSVELFMT_COUNT
};

static const char * MSG_GPSPV_VELFMT_STRING[]=
{
"ECEF",
"ENU"
};

enum MSG_GPSPV_POSFMT
{
GPSPOSFMT_ECEF,
GPSPOSFMT_ENU,
GPSPOSFMT_LLA1,
GPSPOSFMT_LLA2,
GPSPOSFMT_COUNT
};

static const char * MSG_GPSPV_POSFMT_STRING[]=
{
"ECEF",
"ENU",
"LLA",
"LLA"
};

enum MSG_GPSPV_FIXTYPE
{
FIXTYPE_NOFIX,
FIXTYPE_DEADRECKONING,
FIXTYPE_2D,
FIXTYPE_3D,
FIXTYPE_GPSANDDEADRECKONING,
FIXTYPE_COUNT
};

static const char * MSG_GPSPV_FIXTYPE_STRING[]=
{
    "No Fix",
    "Dead Reckoning only",
    "2D Fix",
    "3D Fix",
    "Gps and Dead Reckoning combined"
};

struct msg_GPSPV
{
    double timestamp; // in seconds, to .001
    int week;
    int numOfSV;
    MSG_GPSPV_FIXTYPE FixType;
    bool timeofweekvalid;
    bool weeknumbervalid;
    bool differentialsolution;
    bool gpsfixvalid;
    MSG_GPSPV_POSFMT PositionFormat;
    MSG_GPSPV_VELFMT VelocityFormat;
    bool ENU_PositionRelativeToFirstFix;
    
    double GPS_PosX; // longitude
    double GPS_PosY; // latitude
    double GPS_PosZ; // altitude
    
    double GPS_VelX; // Veast
    double GPS_VelY; // Vnorth
    double GPS_VelZ; // Vup
    
    int DilutionOfPrecision;
    double positionaccuracy;
    double speedaccuracy;
};

#endif
