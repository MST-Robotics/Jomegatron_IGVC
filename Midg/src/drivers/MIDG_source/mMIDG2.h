//mMIDG2.h
//Chad Phillips
//Microbotics, Inc.
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------
//This module contains support for the Microbotics MIDG II INS/GPS.

#ifndef _mMIDG2_Header_
  #define _mMIDG2_Header_

#include "mBin.h"

#ifdef __cplusplus
extern "C" {
#endif

//=====================================================================
//MIDG2 message - Status information
//  STATUS, ID 1, 8 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //Timestamp, GPS time (ms)
  unsigned short Status;    //System status
                            //  8-15:   reserved
                            //     7:   NV configuration invalid
                            //     6:   Timestamp is GPS time
                            //     5:   DGPS
                            //     4:   reserved
                            //   0-3:   Current run mode
                            //          1 = IMU Mode
                            //          2 = VG Initialization
                            //          3 = VG Fast
                            //          4 = VG Medium
                            //          5 = VG Slow
                            //          6 = VG SE
                            //          7 = INS Mode
  short int Temp;           //System temperature (0.01 deg C)
  } mtMIDG2_STATUS;


//=====================================================================
//MIDG2 message - Inertial Measurements (direct from sensors)
//  IMU_DATA, ID 2, 23 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long Time;       //Timestamp, GPS time (ms)
  short int pqr[3];         //Angular rate (0.01 deg/s)
  short int axyz[3];        //Acceleration (mili-g)
  short int mxyz[3];        //Magnetic field
  unsigned char Flags;      //Flags
                            //     7:   GPS 1PPS flag
                            //     6:   Timestamp is GPS time
  } mtMIDG2_IMU_DATA;


//=====================================================================
//MIDG2 message - Magnetometer Measurements
//  IMU_MAG, ID 3, 11 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //Timestamp, GPS time (ms)
  short int mxyz[3];        //Acceleration (mili-g)

  unsigned char Flags;      //Flags
                            //     6:   Timestamp is GPS time
  } mtMIDG2_IMU_MAG;


//=====================================================================
//MIDG2 message - Raw Inertial Measurements (direct from sensors)
//  IMU_RAW, ID 9, 32 bytes
//=====================================================================
//NOTE: this message only comes out during calibration mode.
typedef struct
  {
  unsigned char updated;

  short int pqr[3];         //Angular rate
  short int axyz[3];        //Acceleration
  short int mxyz[3];        //Magnetic field
  unsigned short tcxyz[4];  //temperature
  unsigned short v;         //rail
  unsigned long u;          //count
  } mtMIDG2_IMU_RAW;


//=====================================================================
//MIDG2 message - Navigation Sensor Data (filter results)
//  NAV_SENSOR, ID 10, 39 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long Time;       //Timestamp, GPS time (ms)
  short int pqr[3];         //Angular rate (0.01 deg/s)
  short int axyz[3];        //Acceleration (mili-g)
  short int ypr[3];         //Euler angles, [yaw,pitch,roll] (0.01 deg)
  long int Q[4];            //Quaterion components, scaled 2^30
  unsigned char Flags;      //Flags
                            //     7:   INS Mode
                            //     6:   Timestamp is GPS time
                            //     5:   DGPS
  } mtMIDG2_NAV_SENSOR;


//=====================================================================
//MIDG2 message - Navigation Position and Velocity (filter results)
//  NAV_PV, ID 12, 29 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long Time;       //Timestamp, GPS time (ms)
  long int Pos[3];          //Position
  long int Vel[3];          //Velocity (cm/s)
  unsigned char Details;    //Position/Velocity format
                            //     7:   Solution type
                            //          0 = GPS
                            //          1 = INS
                            //     6:   Timestamp is GPS time
                            //     5:   DGPS
                            //     4:   reserved
                            //   2-3:   Position Format
                            //          0 = ECEF
                            //          1 = ENU
                            //          2,3 = Lon,Lat,Alt
                            //     1:   Velocity Format
                            //          0 = ECEF
                            //          1 = ENU
                            //     0:   ENU position relative to
                            //          0 = Location in NV memory
                            //          1 = First GPS Fix
  } mtMIDG2_NAV_PV;


//=====================================================================
//MIDG2 message - Navigation Heading
//  NAV_HDG, ID 13, 17 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long Time;       //Timestamp, GPS time (ms)
  short int MHdg;           //Magnetic heading (0.01 deg)
  short int MDec;           //Magnetic declimation (0.01 deg)
  short int MDip;           //Magnetic dip (0.01 deg)
  short int COG;            //Course over ground (0.01 deg)
  unsigned short SOG;       //Speed over ground (cm/s)
  short int Vup;            //Vertical speed (cm/s)
  unsigned char Flags;      //Flags
                            //     7:   Declination and dip valid
                            //     6:   Timestamp is GPS time
  } mtMIDG2_NAV_HDG;


//=====================================================================
//MIDG2 message - Navigation Accuracy Estimate
//  NAV_ACC, ID 15, 17 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long Time;       //Timestamp, GPS time (ms)
  unsigned short HPos;      //Horizontal position accuracy (cm)
  unsigned short VPos;      //Vertical position accuracy (cm)
  unsigned short HVel;      //Horizontal velocity accuract (cm/s)
  unsigned short VVel;      //Vertical velocity accuracy (cm/s)
  unsigned short Att;       //Tilt accuracy (0.01 deg)
  unsigned short Hdg;       //Heading accuracy (0.01 deg)

  unsigned char Flags;      //Flags
                            //     7:   INS Mode
                            //     6:   Timestamp is GPS time
                            //     5:   DGPS
                            //     4:   Mag measurement applied
                            //     3:   Ext Hdg measurement applied
  } mtMIDG2_NAV_ACC;



//=====================================================================
//MIDG2 message - GPS Position, Velocity, Time
//  GPS_PV, ID 20, 38 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;
  
  unsigned long  Time;      //GPS time from receiver (ms)
  unsigned short Week;      //GPS week
  unsigned short Details;   //GPS solution details
                            // 12-15:   Number of SVs used in solution
                            //  8-11:   GPS Fix Type
                            //          0 = No Fix
                            //          1 = Dead reckoning only
                            //          2 = 2D Fix
                            //          3 = 3D Fix
                            //          4 = GPS + dead reckoning combined
                            //     7:   Time of week valid
                            //     6:   Week number valid
                            //     5:   Differential solution
                            //     4:   GPS fix valid
                            //          0 = GPS
                            //          1 = INS
                            //   2-3:   Position Format
                            //          0 = ECEF
                            //          1 = ENU
                            //          2,3 = Lon,Lat,Alt
                            //     1:   Velocity Format
                            //          0 = ECEF
                            //          1 = ENU
                            //     0:   ENU position relative to
                            //          0 = Location in NV memory
                            //          1 = First GPS Fix
  long int Pos[3];          //Position
  long int Vel[3];          //Velocity (cm/s)
  unsigned short PDOP;      //Position DOP (0.01 scaling)
  unsigned short PAcc;      //Position accuracy estimate (cm)
  unsigned short SAcc;      //Speed accuracy estimate (cm)
  } mtMIDG2_GPS_PV;




//=====================================================================
//MIDG2 message - GPS Satellite Vehicle Information
//  GPS_SVI, ID 21, 134 bytes (max)
//=====================================================================
typedef struct
  {
  unsigned char ChN;        //channel number, range 0..NCH-1
  unsigned char SVID;       //Satellite ID
  unsigned char CNo;        //Carrier to Noise Ratio (dbHz)
  unsigned char Flags;      //Bitmask
                            //   5-7:   reserved
                            //     4:   SV is unhealthy / not used
                            //     3:   Orbit info is Ephemeris
                            //     2:   Orbit info available for this SV
                            //     1:   DGPS data available for this SV
                            //     0:   SV used for navigation
  char QI;                  //Quality Indicator (0..7)
                            //  7:    C/C locked, receiving 50bps data
                            //  5,6:  Code and Carrier locked
                            //  4:    Code Lock on Signal
                            //  3:    Signal detected but unusable
                            //  1,2:  Channel is searching
                            //  0:    This channel is idle
  char Elev;                //Elevation (degrees)
  short Azim;               //Azimuth (degrees)
  } mtMIDG2_GPS_SVI_blk;

typedef struct
  {
  unsigned char updated;

  unsigned long  Time;            //GPS time from receiver (ms)
  unsigned char  res1;            //reserved
  unsigned char  NCh;             //number of blocks to follow (0..16)
  mtMIDG2_GPS_SVI_blk info[16];   //info for each channel
  } mtMIDG2_GPS_SVI;


//=====================================================================
//MIDG2 message - GPS Raw Measurement Information
//  GPS_RAW, ID 22, X bytes (max)
//=====================================================================
typedef struct
  {
  double CP;                //Carrier phase (cycles)
  double PR;                //Pseudo range (meters)
  float Doppler;            //Doppler (Hz)
  unsigned char SVID;       //Satellite ID
  unsigned char QI;         //Quality Indicator (0..7)
                            //  7:    C/C locked, receiving 50bps data
                            //  5,6:  Code and Carrier locked
                            //  4:    Code Lock on Signal
                            //  3:    Signal detected but unusable
                            //  1,2:  Channel is searching
                            //  0:    This channel is idle
  unsigned char CNo;        //Carrier to Noise Ratio (dbHz)
  unsigned char LLI;        //Loss of Link (RINEX definition)
  } mtMIDG2_GPS_RAW_blk;

typedef struct
  {
  unsigned char updated;

  unsigned long  Time;            //GPS time from receiver (ms)
  unsigned short Week;            //GPS Week number
  unsigned char  res1;            //reserved
  unsigned char  nSVs;            //number of blocks to follow (0..10)
  mtMIDG2_GPS_RAW_blk meas[10];   //info for each channel (upto 10)
  } mtMIDG2_GPS_RAW;


//=====================================================================
//MIDG2 message - GPS Clock Solution
//  GPS_CLK, ID 23, 20 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //Timestamp, GPS time (ms)
  long int CLKB;            //Clock bias (ns)
  long int CLKD;            //Clock drift (ns/s)
  unsigned long TAcc;       //Time accuracy (ns)
  unsigned long FAcc;       //Frequency accuracy (ps/s)
  } mtMIDG2_GPS_CLK;


//=====================================================================
//MIDG2 message - UTC Time
//  TIM_UTC, ID 25, 16 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //Timestamp, GPS time (ms)
  unsigned long TAcc;       //Time accuracy (ns)
  unsigned short Year;      //Year (1999..2099)
  unsigned char Month;      //Month (1..12)
  unsigned char Day;        //Day (1..31)
  unsigned char Hour;       //Hour (0..23)
  unsigned char Min;        //Minute (0..59)
  unsigned char Sec;        //Second (0..59)
  unsigned char Valid;      //Validity
                            //     2:   Valid UTC (leap seconds known)
                            //     1:   GPS Week number valid
                            //     0:   GPS Time of week valid
  } mtMIDG2_TIM_UTC;


//=====================================================================
//MIDG2 message - Time Error Information
//  TIM_ERR, ID 26, 7 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //Timestamp, GPS time (ms)
  char TTB;                 //Time timer bias (clocks)
  char DTB;                 //Data timer bias (clocks)
  unsigned char Flags;      //Flags
                            //     6:   Timestamp is GPS time
  } mtMIDG2_TIM_ERR;


//=====================================================================
//MIDG2 message - Time Pulse Information
//  TIM_PPS, ID 27, 16 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long Time;       //GPS time (ms) of next pulse
  unsigned long Frac;       //Fractional time (ms/2^32) of next pulse
  long int QErr;            //Quantization error (ps)
  unsigned short Week;      //GPS week of next pulse
  unsigned char Flags;      //Flags
                            //  0:  Time base is (0=GPS, 1=UTC)
                            //  1:  UTC is available
  unsigned char res;        //(reserved)
  } mtMIDG2_TIM_PPS;


//=====================================================================
//MIDG2 message - Time Mark Information
//  TIM_TM, ID 28, 8 bytes
//=====================================================================
typedef struct
  {
  unsigned char updated;

  unsigned long  Time;      //GPS time (ms) of received pulse
  unsigned short Week;      //GPS week of received pulse
  unsigned short res;       //(reserved)
  } mtMIDG2_TIM_TM;


//=====================================================================
//=====================================================================
//In order to support multiple parses in use by the same program, a
//structure is created when the parser is instantiated that maintains the
//state information for the parser.  This structure holds the parsed
//messages, the message packetizer, and the receive queue handle.
typedef struct
  {
  //parsed message contents
  mtMIDG2_STATUS         STATUS;
  mtMIDG2_IMU_DATA       IMU_DATA;
  mtMIDG2_IMU_MAG        IMU_MAG;
  mtMIDG2_IMU_RAW        IMU_RAW;
  mtMIDG2_NAV_SENSOR     NAV_SENSOR;
  mtMIDG2_NAV_PV         NAV_PV;
  mtMIDG2_NAV_HDG        NAV_HDG;
  mtMIDG2_NAV_ACC        NAV_ACC;
  mtMIDG2_GPS_PV         GPS_PV;
  mtMIDG2_GPS_SVI        GPS_SVI;
  mtMIDG2_GPS_RAW        GPS_RAW;
  mtMIDG2_GPS_CLK        GPS_CLK;
  mtMIDG2_TIM_UTC        TIM_UTC;
  mtMIDG2_TIM_ERR        TIM_ERR;
  mtMIDG2_TIM_PPS        TIM_PPS;
  mtMIDG2_TIM_TM         TIM_TM;

  //other stuff
  mQueueRH hrMIDG;
  mBinPacketizer Pktzr;

  //For configuration and ACK/NACK, we will want the parser to return the
  //ID, but also have a pointer available for getting the data.  'Message'
  //points to the received packet.  Actually, it points to the ID of the
  //packet.  This is the pointer that is returned by mBinGetPacket.  See
  //mBin.h for details.
  char *Message;
  } mtMIDG2State;


//=====================================================================
//=====================================================================
//mMIDG2Setup initializes the MIDG2 packetizing/parsing subsystem.
//It returns a pointer to a mtMIDG2State structure on success and
//zero on fail.
extern mtMIDG2State *mMIDG2Setup(mQueueRH hrMIDG);

//mMIDG2Parse extracts bytes from the receive queue (specified in the call to
//mMIDG2Setup) looking for valid MIDG II packets.  When a valid packet is
//found, it is stored in the myMIDG2State structure and its ID is returned
//to the user, regardless of whether the queue is empty. (i.e., there may
//be additional packets in the queue after this one).  When the queue is
//empty, this function returns zero.  Call this function until it returns
//zero to keep the queue empty.
extern unsigned char mMIDG2Parse(mtMIDG2State *);

#ifdef __cplusplus
}
#endif
#endif
