#ifndef MIDGPACKETAIDVEL_H
#define MIDGPACKETAIDVEL_H

//#include "midgPacket.h"

struct msg_AIDVEL
{
    //Time, in seconds
    double timestamp;

    //True = GPSTime
    //False = Estimated Delay
    bool timestampType;

    //true = speed
    //false = velocity
    bool speedorvelocity;

    double globalStandardDeviation;
    double velocity;


    double horizontalStandardDeviation;
    double verticalStandardDeviation;
    double velocityx;
    double velocityy;
    double velocityz;

};

#endif
