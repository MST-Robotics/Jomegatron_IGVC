#ifndef MIDGPACKETUTCTIME_H
#define MIDGPACKETUTCTIME_H

struct msg_UTCTIME
{
    double timestamp;
    long nanoseconds;
    unsigned short year; //1999-2099
    unsigned char month; //1-12
    unsigned char day; //1-31
    unsigned char hour; //0-23
    unsigned char minute; //0-59
    unsigned char second; //0-59
    bool validUTC;
    bool weeknumbervalid;
    bool timeofweekvalid;
};

#endif
