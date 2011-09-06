#include <malloc.h>
#include <netinet/in.h>
#include <cstring>
#include <cmath>
#include <cassert>
using namespace std;

#include <ros/ros.h>

#include "midgPacket.h"
#include "fletcher.h"

midgPacket getonepacket( int fd )
{
unsigned char count=1;
bool found = false;
unsigned char next_char=0x00;
    while( count && ! found ){
        ROS_INFO( "gop loop 1" );
        count = read(fd, &next_char, 1 );
        if( count && next_char == 0x81 ){
            ROS_INFO( "gop loop 2" );
            count = read(fd, &next_char, 1 );
            if( count && next_char == 0xA1 ){
                ROS_INFO( "gop loop 3" );
                midgPacket p( fd );
                if( p.passesChecksum( ) ){

                    return p;
                }
            }
        }
    }
    exit(0);
}


midgPacket::midgPacket( int fd )
{
data = NULL;
unsigned char count = 1;
unsigned char next_char = 0x00;
bool found = false;

while( count && !found)
        {
        ROS_INFO("midgpacket loop 1");
        count = read(fd, &next_char, 1 );
        if( count && next_char == 0x81 )
                {
                ROS_INFO("midgpackget loop 2");
                count = read(fd, &next_char, 1 );
                if( count && next_char == 0xA1 )
                        {
                        ROS_INFO("midgpacket loop 3");
                        found = init(fd);
                        }
                }
        }
}

midgPacket::midgPacket( const midgPacket & p )
{
data = NULL;
if( p.data )
    {
    data = (uint8_t*)malloc( p.count );
    memcpy( data, p.data, p.count);
    }

count = p.count;
messageID = p.messageID;
}

bool midgPacket::init( int fd )
{
messageID = MIDG_MESSAGE_INVALID;
count = 0;
data = NULL;

//Do This Thing
if( read( fd, &messageID, 1 ) != 1 )return false;

if( messageID >= MIDG_MESSAGE_COUNT ) return false;


if( read( fd, &count, 1 ) != 1) return false;

data = (uint8_t*)malloc( 2 + count );

if(data == NULL) return false;

//From this point on, any failure must free data ptr. We keep track with the 'failure' bool
bool failure = false;

unsigned int icount = count;
unsigned int imessageID = messageID;
//cout<<"Message:\t"<<messageID<<":\t"<<MIDG_MESSAGE_STRING[messageID]<<endl;

data[0] = messageID;
data[1] = count;

//Start 2 bytes in, because [0] and [1] are hand-packed above
if( read( fd, data+2, count) != count )
    failure = true;

//Read in checksums
if( !failure )
    if(read( fd, &cksum0, 1) != 1 )
        failure = true;

if( !failure )
    if(read( fd, &cksum1, 1) != 1 )
        failure = true;

if( failure )
    {
    free( data );
    return false;
    }
return true;
}

midgPacket::~midgPacket()
{
if( data)
    free(data);
}

bool midgPacket::passesChecksum()
{
fletcher_checksum_t fbuf = fletcher_checksum( data, count+2 );
return (fbuf.first == cksum0) && (fbuf.second == cksum1);
}


//DOCUMENTATION!!?!?!??!
/*
So, what follows is going to be pretty complex.
Each packet contains differing message information,
So there's a function for each message( or will be eventually).
However, it's kind of a pain to read. We have to read to find out
how big the packet is, read that many more bytes, then read the checksums.
That's where these functions take off. Each of extracts information from data.

Prototypes:
returnType midgPacket::handle_msg_MESSAGETYPE()
{
    struct msg_MESSAGETYPE (magical aligned structure)
        {
        //stuff in here is very order and type important
        } __attribute__((__packed__)); //This last line is also super important

    //Endian Conversions!
    ...
    //End endian conversions

    buffer returnType
    //Now we process the data
    return buffer
}
*/

msg_NAVPV midgPacket::handle_msg_NAVPV()
{
    struct internal_msg_NAVPV
    {
    uint8_t message_ID;
    uint8_t count;

    uint32_t timestamp;

    int32_t PosX;
    int32_t PosY;
    int32_t PosZ;

    int32_t VelX;
    int32_t VelY;
    int32_t VelZ;

    uint8_t details;
    } __attribute__((__packed__));

internal_msg_NAVPV * p = (internal_msg_NAVPV *)(data);

//Endian Conversion
p->timestamp = ntohl( p->timestamp );

p->PosX = ntohl( p->PosX );
p->PosY = ntohl( p->PosY );
p->PosZ = ntohl( p->PosZ );

p->VelX = ntohl( p->VelX );
p->VelY = ntohl( p->VelY );
p->VelZ = ntohl( p->VelZ );
//End Endian Conversion

msg_NAVPV buffer;
buffer.timestamp                     = p->timestamp/(double)1000.;

buffer.xVel  =p->VelX / 100.0;
buffer.yVel  =p->VelY / 100.0;
buffer.zVel  =p->VelZ / 100.0;

buffer.xPos = p->PosX / 100.0;
buffer.yPos = p->PosY / 100.0;
buffer.zPos = p->PosZ / 100.0;

buffer.positionvalid                 = (p->details & 0x80) >> 7;
buffer.timestampisGPStime            = (p->details & 0x40) >> 6;
buffer.DGPS                          = (p->details & 0x20) >> 5;
buffer.velocityvalid                 = (p->details & 0x10) >> 4;
buffer.PositionFormat                = (MSG_NAVPV_POSFMT) ( (p->details & 0x0C) >> 2 );
buffer.VelocityFormat                = (MSG_NAVPV_VELFMT) ( (p->details & 0x02) >> 1 );
buffer.ENUPositionRelativeToFirstFix = (p->details & 0x01) >> 0;

//cout<<"\tPosValid:"<<buffer.positionvalid<<endl;
//cout<<"\tVelValid:"<<buffer.velocityvalid<<endl;
//cout<<"\tDGPS:"<<buffer.DGPS<<endl;
//cout<<"\tPositionFormat:"<<MSG_NAVPV_POSFMT_STRING[buffer.PositionFormat]<<endl;
//cout<<"\tVelocityFormat:"<<MSG_NAVPV_VELFMT_STRING[buffer.VelocityFormat]<<endl;
//cout<<"\tPos(XYZ):"<<buffer.xPos/100<<'\t'<<buffer.yPos/100<<','<<buffer.zPos/100<<endl;

if(/*buffer.positionvalid &&*/ startdata.gotafix==false )
    {
    startdata.gotafix = true;
    startdata.x = buffer.xPos;
    startdata.y = buffer.yPos;
    startdata.z = buffer.zPos;
    }

if( startdata.gotafix)
    {
    buffer.xPos -= startdata.x;
    buffer.yPos -= startdata.y;
    buffer.zPos -= startdata.z;
    }

//if( buffer.positionvalid)
//cout<<"\tPOS:"<<buffer.xPos<<'\t'<<buffer.yPos<<'\t'<<buffer.zPos<<'\t'<<getCurTime()<<endl;

//if(buffer.velocityvalid)
//cout<<"\tVEL:"<<buffer.xVel<<'\t'<<buffer.yVel<<'\t'<<buffer.zVel<<'\t'<<getCurTime()<<endl;

return buffer;
}

msg_NAVHDG midgPacket::handle_msg_NAVHDG( )
{
struct internal_msg_NAVHDG
    {
    uint8_t messageID;
    uint8_t count;
    uint32_t timestamp;
    int16_t magHDG;
    int16_t magDecl;
    int16_t magDip;
    int16_t cog;
    uint16_t sog;
    int16_t vup;
    uint8_t flags;
    }__attribute__((__packed__));

internal_msg_NAVHDG * p = (internal_msg_NAVHDG *)(data);

//Endian Conversions
p->timestamp = ntohl( p->timestamp );
p->magHDG = ntohs( p->magHDG );
p->magDecl = ntohs( p->magDecl );
p->magDip = ntohs( p->magDip );
p->cog = ntohs( p->cog );
p->sog = ntohs( p->sog );
p->vup = ntohs( p->vup );
//End Endian Conversions, now we do real work

msg_NAVHDG buffer;

//convert to seconds
buffer.timestamp = (double)p->timestamp/1000.;

//convert to degrees
buffer.magHeading = p->magHDG / 100.;
buffer.magDeclination = p->magDecl / 100.;
buffer.magDip = p->magDip / 100.;



//cout<<"DegreeMagHEADING:"<< buffer.magHeading<<endl;

//convert to radians
buffer.magHeading     *= ( M_PI / 180 );
buffer.magDeclination *= ( M_PI / 180 );
buffer.magDip         *= ( M_PI / 180 );

//convert to degrees, then radians
buffer.cog = p->cog / 100. * ( M_PI / 180 );

//convert to m/s
buffer.sog = p->sog / 100.;
buffer.vup = p->vup / 100.;

//These next two are really the same thing
buffer.declinationvalid = (p->flags & 0x80)>>7;
buffer.dipvalid         = (p->flags & 0x80)>>7;
buffer.timestampisGPStime = (p->flags & 0x40)>>6;

//cout<<"\tmagHeading:"<<buffer.magHeading<<endl;

//INSCourse is actually ground track, which is not what we need.
//cout<<"\tINSCourse?:"<<buffer.cog<<endl;

return buffer;

}

/* IMU_MAG Packet */
msg_IMUMAG midgPacket::handle_msg_IMUMAG( )
{
struct internal_msg_IMUMAG
    {
	uint8_t messageID;
	uint8_t count;
	uint32_t timestamp;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	uint8_t flags;
    }__attribute__((__packed__));

internal_msg_IMUMAG * p = (internal_msg_IMUMAG *)(data);

//Endian Conversions
p->timestamp = ntohl( p->timestamp );
p->mag_x = ntohs( p->mag_x );
p->mag_y = ntohs( p->mag_y );
p->mag_z = ntohs( p->mag_z );
//End Endian Conversions, now we do real work

msg_IMUMAG buffer;

//convert to seconds
buffer.timestamp = (double)p->timestamp/1000.;

//convert to degrees

//buffer.mag_xz_rads = p->mag_z;
buffer.mag_y_sine = (p->mag_y+1000)/1000.; //shift sine wave to 0
//buffer.mag_z_sine = (p->mag_z+4321-6315;
buffer.mag_x_sine = (p->mag_x-1788)/1000.;

return buffer;

}




/****************/
msg_STATUS midgPacket::handle_msg_STATUS( )
{
    struct  internal_msg_STATUS
    {
    uint8_t messageID;
    uint8_t count;
    uint32_t timestamp;
    uint16_t status;
    int16_t temperature;
    } __attribute__((__packed__));

internal_msg_STATUS * p = (internal_msg_STATUS *)(data);

//Endian Conversions
p->timestamp = ntohl(p->timestamp);
p->status = ntohs(p->status);
p->temperature = ntohs(p->temperature);
//End Endian Conversions

msg_STATUS buffer;

//convert to seconds
buffer.timestamp = (double)p->timestamp/1000.;

buffer.nonvolConfigValid  = (0x0080 & p->status ) >> 7;
buffer.timestampisGPStime = (0x0040 & p->status ) >> 6;
buffer.DGPS               = (0x0020 & p->status ) >> 5;
buffer.currentmode        = (MSG_STATUS_MODE)( (0x000F & p->status ) >> 0 );

//convert to degrees C
buffer.temperature = p->temperature/(double)100;

//cout<<"\tMode:" << buffer.currentmode << endl;
//cout<<"\tDGPS:"<<buffer.DGPS<<endl;
//cout<<"\tTSGPDT?"<<buffer.timestampisGPStime<<endl;
//cout<<"\tNVV?"<<buffer.nonvolConfigValid<<endl;

//cout<<"\ttemp:"<<buffer.temperature<<" degrees Centigrade"<<endl;

//cout<<endl;
}

msg_NAVSENSE midgPacket::handle_msg_NAVSENSE()
{
    struct internal_msg_NAVSENSE
    {
    uint8_t messageID;
    uint8_t count;
    uint32_t timestamp;

    int16_t xAngRate;
    int16_t yAngRate;
    int16_t zAngRate;

    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;

    int16_t yaw;
    int16_t pitch;
    int16_t roll;

    int32_t Qw;
    int32_t Qx;
    int32_t Qy;
    int32_t Qz;
    uint8_t flags;
    } __attribute__((__packed__));

internal_msg_NAVSENSE * p = (internal_msg_NAVSENSE *)data;

//Endian Conversion Time
p->timestamp = ntohl(p->timestamp);

p->xAngRate = ntohs( p->xAngRate );
p->yAngRate = ntohs( p->yAngRate );
p->zAngRate = ntohs( p->zAngRate );

p->xAccel = ntohs( p->xAccel);
p->yAccel = ntohs( p->yAccel);
p->zAccel = ntohs( p->zAccel);

p->yaw = ntohs( p->yaw );
p->pitch = ntohs( p->pitch );
p->roll = ntohs( p->roll );

p->Qw = ntohl( p->Qw );
p->Qx = ntohl( p->Qx );
p->Qy = ntohl( p->Qy );
p->Qz = ntohl( p->Qz );
// End of Endian Conversions, can now use data

msg_NAVSENSE buffer;

//convert to seconds
buffer.timestamp = (double)p->timestamp/1000.;

//convert to degrees/second
buffer.xAngRate = (double)p->xAngRate/100;
buffer.yAngRate = (double)p->yAngRate/100;
buffer.zAngRate = (double)p->zAngRate/100;

//convert to meters/(second^2)
buffer.xAccel = p->xAccel*(double)9.8/1000.;
buffer.yAccel = p->yAccel*(double)9.8/1000.;
buffer.zAccel = p->zAccel*(double)9.8/1000.;

//Convert to degrees, then radians
buffer.yaw = (double)p->yaw/100.*(M_PI/180);
buffer.pitch = (double)p->pitch/100.*(M_PI/180);
buffer.roll = (double)p->roll/100. *(M_PI/180);

//convert to unit quaternion
const double QSCALER = pow((long double)2,-30);
buffer.Qw = (double)p->Qw*QSCALER;
buffer.Qx = (double)p->Qx*QSCALER;
buffer.Qy = (double)p->Qy*QSCALER;
buffer.Qz = (double)p->Qz*QSCALER;

//cout<<"\tAccel(XYZ):"<<buffer.xAccel<<'\t'<<buffer.yAccel<<'\t'<<buffer.zAccel<<'\t'<<getCurTime()<<endl;
//cout<<"\tARATE:"<<buffer.xAngRate<<'\t'<<buffer.yAngRate<<'\t'<<buffer.zAngRate<<'\t'<<getCurTime()<<endl;
return buffer;
}

msg_GPSPV midgPacket::handle_msg_GPSPV()
{
    struct internal_msg_GPSPV
    {
        uint8_t messageID;
        uint8_t count;
        uint32_t GPS_ts;
        uint16_t GPS_week;
        uint16_t Details;
        int32_t GPS_PosX;
        int32_t GPS_PosY;
        int32_t GPS_PosZ;
        int32_t GPS_VelX;
        int32_t GPS_VelY;
        int32_t GPS_VelZ;
        uint16_t PDOP;
        uint16_t PAcc;
        uint16_t SAcc;
    }__attribute__((__packed__));

    internal_msg_GPSPV * p = (internal_msg_GPSPV *)(data);

    // Endian Conversion
    p->GPS_ts = ntohl(p->GPS_ts);
    p->GPS_week = ntohs(p->GPS_week);
    p->Details = ntohs(p->Details);
    p->GPS_PosX = ntohl(p->GPS_PosX);
    p->GPS_PosY = ntohl(p->GPS_PosY);
    p->GPS_PosZ = ntohl(p->GPS_PosZ);
    p->GPS_VelX = ntohl(p->GPS_VelX);
    p->GPS_VelY = ntohl(p->GPS_VelY);
    p->GPS_VelZ = ntohl(p->GPS_VelZ);
    p->PDOP = ntohs(p->PDOP);
    p->PAcc = ntohs(p->PAcc);
    p->SAcc = ntohs(p->SAcc);
    // End endian conversion

    msg_GPSPV buffer;

    buffer.timestamp = (double)(p->GPS_ts/1000.0);
    buffer.week = (int)(p->GPS_week);

    buffer.numOfSV = ((p->Details & 0xF000) >> 12);
    buffer.FixType = (MSG_GPSPV_FIXTYPE)((p->Details & 0x0F00) >> 8 );
    buffer.timeofweekvalid = (p->Details & 0x0080 ) >> 7;
    buffer.weeknumbervalid = (p->Details & 0x0040 ) >> 6;
    buffer.differentialsolution = (p->Details & 0x0020 ) >> 5;
    buffer.gpsfixvalid = (p->Details & 0x0010 ) >> 4;
    buffer.PositionFormat = (MSG_GPSPV_POSFMT)((p->Details & 0x000C ) >> 2);
    buffer.VelocityFormat = (MSG_GPSPV_VELFMT)((p->Details & 0x0002 ) >> 1);
    buffer.ENU_PositionRelativeToFirstFix = (p->Details & 0x0001 ) >> 0;

    buffer.GPS_PosX = (double)(p->GPS_PosX*0.0000001); // longitude in 10^-7 degrees
    buffer.GPS_PosY = (double)(p->GPS_PosY*0.0000001); // latitude in 10^-7 degrees
    buffer.GPS_PosZ = (double)(p->GPS_PosZ/100.0);

    buffer.GPS_VelX = (double)(p->GPS_VelX/100.0);
    buffer.GPS_VelY = (double)(p->GPS_VelY/100.0);
    buffer.GPS_VelZ = (double)(p->GPS_VelZ/100.0);

    buffer.DilutionOfPrecision = (int)(p->PDOP);
    buffer.positionaccuracy = (double)(p->PAcc/100.0);
    buffer.speedaccuracy = (double)(p->SAcc/100.0);

    return buffer;
}

msg_GPSCLOCK midgPacket::handle_msg_GPSCLOCK()
{
    struct internal_msg_GPSCLOCK
    {
        uint8_t messageID;
        uint8_t count;
        uint32_t GPS_ts;
        int32_t CLKB;
        int32_t CLKD;
        uint32_t TAcc;
        uint32_t FAcc;
    }__attribute__((__packed__));

    internal_msg_GPSCLOCK * p = (internal_msg_GPSCLOCK *)(data);

    // Endian Conversion
    p->GPS_ts = ntohl(p->GPS_ts);
    p->CLKB = ntohl(p->CLKB);
    p->CLKD = ntohl(p->CLKD);
    p->TAcc = ntohl(p->TAcc);
    p->FAcc = ntohl(p->FAcc);
    // End Endian Conversion

    msg_GPSCLOCK buffer;

    buffer.timestamp = (double)(p->GPS_ts/100.0);
    buffer.clock_bias = (long)(p->CLKB);
    buffer.clock_drift = (long)(p->CLKD);
    buffer.time_accuracy_estimate = (long)(p->TAcc);
    buffer.frequency_accuracy_estimate = (long)(p->FAcc);

    return buffer;
}

msg_UTCTIME midgPacket::handle_msg_UTCTIME()
{
    struct internal_msg_UTCTIME
    {
        uint8_t messageID;
        uint8_t count;
        uint32_t GPS_ts;
        int32_t Nano;
        uint16_t Year;
        uint8_t Month;
        uint8_t Day;
        uint8_t Hour;
        uint8_t Min;
        uint8_t Sec;
        uint8_t Details;
    }__attribute__((__packed__));

    internal_msg_UTCTIME * p = (internal_msg_UTCTIME *)(data);

    // Endian Conversion
    p->GPS_ts = ntohl(p->GPS_ts);
    p->Nano = ntohl(p->Nano);
    p->Year = ntohs(p->Year);
    // End Endian Conversion

    msg_UTCTIME buffer;

    buffer.timestamp = (double)(p->GPS_ts/100.0) ;
    buffer.nanoseconds = (long)(p->Nano);
    buffer.year = (unsigned short)(p->Year);
    buffer.month = (unsigned char)(p->Month);
    buffer.day = (unsigned char)(p->Day);
    buffer.hour = (unsigned char)(p->Hour);
    buffer.minute = (unsigned char)(p->Min);
    buffer.second = (unsigned char)(p->Sec);
    buffer.validUTC = ((p->Details & 0x04) >> 2);
    buffer.weeknumbervalid = ((p->Details & 0x02) >> 1);
    buffer.timeofweekvalid = ((p->Details & 0x01) >> 0);

    return buffer;
}
/*
void send_msg_AIDVEL( int fd, msg_AIDVEL input )
{
struct internal_msg_AIDVEL
    {
    uint8_t messageID;
    uint8_t count;
    uint32_t timestamp;

    //magic bitstructure packing
    uint16_t magic;
    //end magic bitstructure packing

    //velocities (or just use upVel if a speed)
    int16_t upVel;
    int16_t eastVel;
    int16_t northVel;

    uint16_t horizontalStD;
    } __attribute__((__packed__)) ;

//cout<<"\tmagic struct size:"<<sizeof(internal_msg_AIDVEL)<<endl;
//cout<<"\tTotal size out should be:"<<sizeof(internal_msg_AIDVEL)+4<<endl;

internal_msg_AIDVEL buffer;
memset( &buffer, 0x00, sizeof(buffer) );

buffer.messageID = 38;
buffer.count     = 14;
buffer.timestamp = (uint32_t)(input.timestamp*1000);
if( input.speedorvelocity )
    {
    buffer.upVel = htons((int16_t)(input.velocity * 100));
    buffer.magic = 0;
    if( input.timestampType )
        buffer.magic |= 0x8000;

    if( input.speedorvelocity )
        buffer.magic |= 0x4000;

    uint16_t standardDeviationBuffer = (uint16_t)(input.globalStandardDeviation * 10);
    buffer.magic |= ( 0x0FFF & standardDeviationBuffer );

    buffer.magic = htons( buffer.magic );
    }
else
    {
    assert(false); //This section not done yet
    }



//Add 2 bytes for sync bytes
//Add 2 bytes for 16 bit fletcher checksum
const int FULLSIZE = 2 + sizeof(internal_msg_AIDVEL) + 2;
uint8_t * rawbuffer = (uint8_t * )malloc( FULLSIZE );
memset( rawbuffer, 0x00, FULLSIZE);
rawbuffer[0]=0x81;
rawbuffer[1]=0xA1;

assert( sizeof(buffer) <= ( FULLSIZE - 4) );

memcpy( rawbuffer + 2, &buffer, sizeof(buffer) );
fletcher_checksum_t fbuf = fletcher_checksum( &buffer, sizeof(buffer) );
rawbuffer[FULLSIZE-2] = fbuf.first;
rawbuffer[FULLSIZE-1] = fbuf.second;

//write( fd, rawbuffer, FULLSIZE );

//debugging
FILE*fp ;fp = fopen ( "magic.bin","w+" );if( fp == NULL ){puts ( "cannot open file");exit( 1 ) ;}
//cout<<"Wrote:"<<fwrite ( rawbuffer, 1, FULLSIZE, fp )<<"bytes"<<endl;
fclose( fp );
//end debugging

free( rawbuffer );
}
*/

