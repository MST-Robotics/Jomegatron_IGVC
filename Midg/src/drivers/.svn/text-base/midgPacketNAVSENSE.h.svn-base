#ifndef MIDGPACKETNAVSENSE_H
#define MIDGPACKETNAVSENSE_H

//#include "midgPacket.h"

struct msg_NAVSENSE
{
    //Time, in seconds
    double timestamp;

    //Degrees/Second, resolution to .01
    double xAngRate;
    double yAngRate;
    double zAngRate;

    //Acceleration in meters/second
    double xAccel;
    double yAccel;
    double zAccel;

    //YPR in degrees, resolution to .01
    double yaw;
    double pitch;
    double roll;

    //Unit Quaternion
    double Qw;
    double Qx;
    double Qy;
    double Qz;
};

#endif
