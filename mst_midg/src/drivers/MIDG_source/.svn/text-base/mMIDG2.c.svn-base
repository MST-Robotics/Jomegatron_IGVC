//mMIDG2.c
//Chad Phillips
//Microbotics, Inc.
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------

#include <malloc.h>
#include "mMIDG2.h"

//local routines used to parse the MIDG2 messages
static void StoreSTATUS(mtMIDG2_STATUS *p, unsigned char *buf);
static void StoreIMU_DATA(mtMIDG2_IMU_DATA *p, unsigned char *buf);
static void StoreIMU_MAG(mtMIDG2_IMU_MAG *p, unsigned char *buf);
static void StoreIMU_RAW(mtMIDG2_IMU_RAW *p, unsigned char *buf);
static void StoreNAV_SENSOR(mtMIDG2_NAV_SENSOR *p, unsigned char *buf);
static void StoreNAV_PV(mtMIDG2_NAV_PV *p, unsigned char *buf);
static void StoreNAV_HDG(mtMIDG2_NAV_HDG *p, unsigned char *buf);
static void StoreNAV_ACC(mtMIDG2_NAV_ACC *p, unsigned char *buf);
static void StoreGPS_PV(mtMIDG2_GPS_PV *p, unsigned char *buf);
static void StoreGPS_SVI(mtMIDG2_GPS_SVI *p, unsigned char *buf);
static void StoreGPS_RAW(mtMIDG2_GPS_RAW *p, unsigned char *buf);
static void StoreGPS_CLK(mtMIDG2_GPS_CLK *p, unsigned char *buf);
static void StoreTIM_UTC(mtMIDG2_TIM_UTC *p, unsigned char *buf);
static void StoreTIM_ERR(mtMIDG2_TIM_ERR *p, unsigned char *buf);
static void StoreTIM_PPS(mtMIDG2_TIM_PPS *p, unsigned char *buf);
static void StoreTIM_TM(mtMIDG2_TIM_TM *p, unsigned char *buf);


//===================================================================
//                       mMIDG2Setup
//===================================================================
//mMIDG2Setup stores the queue handles that are to be used for talking
//to the receiver.  Also, it creates a mBin packetizer on the recieve queue.
mtMIDG2State *mMIDG2Setup(mQueueRH hr)
  {
  mtMIDG2State *p;

  //Create a new state variable.
  p = (mtMIDG2State *)malloc(sizeof(mtMIDG2State));
  if(!p)
    return(0);

  //zero the data in the new state structure
  memset((void *)p, 0x00, sizeof(mtMIDG2State));

  //Fill in the relevant data.
  p->hrMIDG = hr;
  p->Pktzr=mBinCreatePacketizer(p->hrMIDG, 256);
  if(!(p->Pktzr))
    {
    free(p);
    return(0);
    }
  return(p);
  }

//===================================================================
//                       mMIDG2Parse
//===================================================================
//mMIDG2Parse parses all the data in the MIDG receive queue.  When a
//message is found, the data is stored and the message numer is returned.
//When the queue is empty, this function returns zero.  Call this function
//until it returns zero to keep the queue empty.
unsigned char mMIDG2Parse(mtMIDG2State *p)
  {
  unsigned char *buf, ID;

  //get a packet is one is available
  buf=mBinGetPacket(p->Pktzr);

  //if no packet is available, return zero.
  if(!buf)
    return(0);

  //The buffer contents start with:
  //  ID (uc)
  //  Payload Length (uc, bytes)
  //  Payload (all elements are big-endian)
  //Lets call the appropriate storage function  based on the message ID. 
  //We will not handle the length here, but will provide a pointer 
  //to the length to each storage function.
  p->Message = buf;
  ID = *buf++;
  
  switch(ID)
    {
    case 1:  StoreSTATUS(&(p->STATUS), buf);          break;
    case 2:  StoreIMU_DATA(&(p->IMU_DATA), buf);      break;
    case 3:  StoreIMU_MAG(&(p->IMU_MAG), buf);        break;
    case 9:  StoreIMU_RAW(&(p->IMU_RAW), buf);        break;
    case 10: StoreNAV_SENSOR(&(p->NAV_SENSOR), buf);  break;
    case 12: StoreNAV_PV(&(p->NAV_PV), buf);          break;
    case 13: StoreNAV_HDG(&(p->NAV_HDG), buf);        break;
    case 15: StoreNAV_ACC(&(p->NAV_ACC), buf);        break;
    case 20: StoreGPS_PV(&(p->GPS_PV), buf);          break;
    case 21: StoreGPS_SVI(&(p->GPS_SVI), buf);        break;
    case 22: StoreGPS_RAW(&(p->GPS_RAW), buf);        break;
    case 23: StoreGPS_CLK(&(p->GPS_CLK), buf);        break;
    case 25: StoreTIM_UTC(&(p->TIM_UTC), buf);        break;
    case 26: StoreTIM_ERR(&(p->TIM_ERR), buf);        break;
    case 27: StoreTIM_PPS(&(p->TIM_PPS), buf);        break;
    case 28: StoreTIM_TM(&(p->TIM_TM), buf);          break;
    }

  //return the ID/Class word
  return(ID);
  }


//===================================================================
//                       StoreSTATUS
//===================================================================
//Store STATUS message in the supplied structure mMIDG2_STATUS
static void StoreSTATUS(mtMIDG2_STATUS *p, unsigned char *buf)
  {
  //skip the length as the STATUS message is always 8 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);    //GPS Milisecond Time of Week
  p->Status=mBinGetUShort(&buf); //System status
                                 //  8-15:   reserved
                                 //     7:   NV configuration invalid
                                 //     6:   Timestamp is GPS time
                                 //     5:   DGPS
                                 //     4:   reserved
                                 //   0-3:   Current run mode
                                 //          1 = IMU Mode
                                 //          2 = Vertical Gyro (VG) Mode
                                 //          3 = INS Mode
  p->Temp=mBinGetShort(&buf);    //System temperature (0.01 deg C)                            

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreIMU_DATA
//===================================================================
//Store IMU_DATA message in the supplied structure mMIDG2_IMU_DATA
static void StoreIMU_DATA(mtMIDG2_IMU_DATA *p, unsigned char *buf)
  {
  int i;
  
  //skip the length as the IMU_DATA message is always 22 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);      //GPS Milisecond Time of Week

  for(i=0;i<3;++i)
    p->pqr[i]=mBinGetShort(&buf);  //Angular rate (0.01 deg/s)
  for(i=0;i<3;++i)
    p->axyz[i]=mBinGetShort(&buf); //Acceleration (mili-g)
  for(i=0;i<3;++i)
    p->mxyz[i]=mBinGetShort(&buf); //Magnetic field

  p->Flags=*buf++;                 //Flags
                                   //  7: GPS 1PPS flag
                                   //  6: Timestamp is GPS time

  //Mark message as having been updated.
  p->updated=1;
  }


//===================================================================
//                       StoreIMU_MAG
//===================================================================
//Store IMU_MAG message in the supplied structure mMIDG2_IMU_MAG
static void StoreIMU_MAG(mtMIDG2_IMU_MAG *p, unsigned char *buf)
  {
  int i;
  
  //skip the length as the IMU_MAG message is always 10 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);       //GPS Milisecond Time of Week
  for(i=0;i<3;++i)
    p->mxyz[i]=mBinGetShort(&buf);  //Magnetic field

  p->Flags=*buf++;                  //Flags
                                    //  6: Timestamp is GPS time

  //Mark message as having been updated.
  p->updated=1;
  }


//===================================================================
//                       StoreIMU_RAW
//===================================================================
//Store IMU_RAW message in the supplied structure mMIDG2_IMU_RAW.
//This message is only available in calibration mode.
static void StoreIMU_RAW(mtMIDG2_IMU_RAW *p, unsigned char *buf)
  {
  int i;

  //skip the length as the IMU_RAW message is always 32 bytes.
  ++buf;
  
  for(i=0;i<3;++i)
    p->pqr[i]=mBinGetShort(&buf);
  for(i=0;i<3;++i)
    p->axyz[i]=mBinGetShort(&buf);
  for(i=0;i<3;++i)
    p->mxyz[i]=mBinGetShort(&buf);
  for(i=0;i<4;++i)
    p->tcxyz[i]=mBinGetUShort(&buf);
  p->v=mBinGetUShort(&buf);
  p->u=mBinGetULong(&buf);

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreNAV_SENSOR
//===================================================================
//Store NAV_SENSOR message in the supplied structure mMIDG2_NAV_SENSOR
static void StoreNAV_SENSOR(mtMIDG2_NAV_SENSOR *p, unsigned char *buf)
  {
  int i;
  
  //skip the length as the NAV_SENSOR message is always 38 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);      //GPS Milisecond Time of Week

  for(i=0;i<3;++i)
    p->pqr[i]=mBinGetShort(&buf);  //Angular rate (0.01 deg/s)
  for(i=0;i<3;++i)
    p->axyz[i]=mBinGetShort(&buf); //Acceleration (mili-g)
  for(i=0;i<3;++i)
    p->ypr[i]=mBinGetShort(&buf);  //Euler angles (0.01 deg)
  for(i=0;i<4;++i)
    p->Q[i]=mBinGetLong(&buf);     //Quat, scaled 2^30

  p->Flags=*buf++;                 //Flags
                                   //  7: INS Mode
                                   //  6: Timestamp is GPS time
                                   //  5: DGPS

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreNAV_PV
//===================================================================
//Store NAV_PV message in the supplied structure mMIDG2_NAV_PV
static void StoreNAV_PV(mtMIDG2_NAV_PV *p, unsigned char *buf)
  {
  int i;
  
  //skip the length as the NAV_PV message is always 8 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);    //GPS Milisecond Time of Week
  for(i=0;i<3;++i)
    p->Pos[i]=mBinGetLong(&buf); //Position
  for(i=0;i<3;++i)
    p->Vel[i]=mBinGetLong(&buf); //Velocity (cm/s)
  p->Details=*buf;               //Position/Velocity format
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

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreNAV_HDG
//===================================================================
//Store NAV_HDG message in the supplied structure mMIDG2_NAV_HDG
static void StoreNAV_HDG(mtMIDG2_NAV_HDG *p, unsigned char *buf)
  {
  //skip the length as the NAV_HDG message is always 17 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);    //GPS Milisecond Time of Week

  p->MHdg=mBinGetShort(&buf);    //Magnetic heading (0.01 deg)
  p->MDec=mBinGetShort(&buf);    //Magnetic declination (0.01 deg)
  p->MDip=mBinGetShort(&buf);    //Magnetic dip (0.01 deg)
  p->COG=mBinGetShort(&buf);     //Course over ground (0.01 deg)
  p->SOG=mBinGetUShort(&buf);    //Speed over ground (cm/s)
  p->Vup=mBinGetShort(&buf);     //Vertical Velocity (cm/s)

  p->Flags=*buf++;               //Flags
                                 //  7: Magnetic declination and dip valid
                                 //  6: Timestamp is GPS time

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreNAV_ACC
//===================================================================
//Store NAV_ACC message in the supplied structure mMIDG2_NAV_ACC
static void StoreNAV_ACC(mtMIDG2_NAV_ACC *p, unsigned char *buf)
  {
  //skip the length as the NAV_ACC message is always 17 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);    //GPS Milisecond Time of Week

  p->HPos=mBinGetUShort(&buf);   //Horizontal Pos Accuracy (cm)
  p->VPos=mBinGetUShort(&buf);   //Vertical Pos Accuracy (cm)

  p->HVel=mBinGetUShort(&buf);   //Horizontal Vel Accuracy (cm/s)
  p->VVel=mBinGetUShort(&buf);   //Vertical Vel Accuracy (cm/s)
  p->Att=mBinGetUShort(&buf);    //Tilt Accuracy (0.01 deg)
  p->Hdg=mBinGetUShort(&buf);    //Heading Accuracy (0.01 deg)

  p->Flags=*buf++;               //Flags
                                 //  7: Content valid
                                 //  6: Timestamp is GPS time
                                 //  5: DGPS

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreGPS_PV
//===================================================================
//Store GPS_PV message in the supplied structure mMIDG2_GPS_PV
static void StoreGPS_PV(mtMIDG2_GPS_PV *p, unsigned char *buf)
  {
  int i;
  
  //skip the length as the GPS_PV message is always 38 bytes.
  ++buf;
  
  p->Time=mBinGetULong(&buf);      //GPS Milisecond Time of Week
  p->Week=mBinGetUShort(&buf);     //GPS time of week
  p->Details=mBinGetUShort(&buf);  //GPS solution details
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
  for(i=0;i<3;++i)
    p->Pos[i]=mBinGetLong(&buf);   //Position
  for(i=0;i<3;++i)
    p->Vel[i]=mBinGetLong(&buf);   //Velocity (cm/s)
  p->PDOP=mBinGetUShort(&buf);     //Position DOP (0.01 scaling)
  p->PAcc=mBinGetUShort(&buf);     //Position accuracy estimate (cm)
  p->SAcc=mBinGetUShort(&buf);     //Speed accuracy estimate (cm)

  //Mark message as having been updated.
  p->updated=1;
  }



//===================================================================
//                       StoreGPS_SVI
//===================================================================
//Store GPS_SVI message in the supplied structure mMIDG2_GPS_SVI
static void StoreGPS_SVI(mtMIDG2_GPS_SVI *p, unsigned char *buf)
  {
  int i;
  
  //skip the length. The GPS_SVI message will depend on NCh.
  ++buf;
  
  p->Time=mBinGetULong(&buf);     //GPS Milisecond Time of Week
  p->res1=*buf++;                 //reserved
  p->NCh=*buf++;                  //blocks to follow (0..16)
  if(p->NCh > 16)                 //be safe
    p->NCh = 16;
  for(i=0;i<p->NCh;++i)
    {
    p->info[i].ChN=*buf++;        //channel number, range 0..NCH-1
    p->info[i].SVID=*buf++;       //Satellite ID
    p->info[i].CNo=*buf++;        //Carrier to Noise Ratio (dbHz)
    p->info[i].Flags=*buf++;      //Bitmask
                                  //   5-7:   reserved
                                  //     4:   SV is unhealthy / not used
                                  //     3:   Orbit info is Ephemeris
                                  //     2:   Orbit info available for this SV
                                  //     1:   DGPS data available for this SV
                                  //     0:   SV used for navigation
    p->info[i].QI=*buf++;         //Quality Indicator (0..7)
                                  //  7:    C/C locked, receiving 50bps data
                                  //  5,6:  Code and Carrier locked
                                  //  4:    Code Lock on Signal
                                  //  3:    Signal detected but unusable
                                  //  1,2:  Channel is searching
                                  //  0:    This channel is idle
    p->info[i].Elev=*buf++;       //Elevation (degrees)
    p->info[i].Azim=mBinGetShort(&buf);  //Azimuth (degrees)
    }
  
  //Mark message as having been updated.
  p->updated=1;
  }


//===================================================================
//                       StoreGPS_RAW
//===================================================================
//Store GPS_RAW message in the supplied structure mMIDG2_GPS_RAW
static void StoreGPS_RAW(mtMIDG2_GPS_RAW *p, unsigned char *buf)
  {
  int i;

  //skip the length. The GPS_RAW message will depend on NCh.
  ++buf;

  p->Time=mBinGetULong(&buf);     //GPS Milisecond Time of Week
  p->Week=mBinGetUShort(&buf);    //GPS Week
  p->res1=*buf++;                 //reserved
  p->nSVs=*buf++;                 //blocks to follow (0..10)
  if(p->nSVs > 10)                //be safe
    p->nSVs = 10;
  for(i=0;i<p->nSVs;++i)
    {
    p->meas[i].CP=mBinGetDouble(&buf);    //Carrier phase (cycles)
    p->meas[i].PR=mBinGetDouble(&buf);    //Pseudo range (meters)
    p->meas[i].Doppler=mBinGetFloat(&buf);//Doppler (Hz)
    p->meas[i].SVID=*buf++;       //Satellite ID
    p->meas[i].QI=*buf++;         //Quality Indicator (0..7)
                                  //  7:    C/C locked, receiving 50bps data
                                  //  5,6:  Code and Carrier locked
                                  //  4:    Code Lock on Signal
                                  //  3:    Signal detected but unusable
                                  //  1,2:  Channel is searching
                                  //  0:    This channel is idle
    p->meas[i].CNo=*buf++;        //Carrier to Noise Ratio (dbHz)
    p->meas[i].LLI=*buf++;        //Link loss (RINEX definition)
    }

  //Mark message as having been updated.
  p->updated=1;
  }


//===================================================================
//                       StoreGPS_CLK
//===================================================================
//Store GPS_CLK message in the supplied structure mMIDG2_GPS_CLK
static void StoreGPS_CLK(mtMIDG2_GPS_CLK *p, unsigned char *buf)
  {
  //skip the length as the GPS_CLK message is always 20 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);   //GPS Milisecond Time of Week (ms)
  p->CLKB=mBinGetLong(&buf);    //Clock bias (ns)
  p->CLKD=mBinGetLong(&buf);    //Clock drift (ns/s)
  p->TAcc=mBinGetULong(&buf);   //Time accuracy estimate (ns)
  p->FAcc=mBinGetULong(&buf);   //Frequency accuracy estimate (ps/s)

  //Mark message as having been updated.
  p->updated=1;
  }

//===================================================================
//                       StoreTIM_UTC
//===================================================================
//Store TIM_UTC message in the supplied structure mMIDG2_TIM_UTC
static void StoreTIM_UTC(mtMIDG2_TIM_UTC *p, unsigned char *buf)
  {
  //skip the length as the TIM_UTC message is always 16 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);   //GPS Milisecond Time of Week (ms)
  p->TAcc=mBinGetULong(&buf);   //Time accuracy estimate (ns)
  p->Year=mBinGetUShort(&buf);  //Year (1999..2099)
  p->Month=*buf++;              //Month (1..12)
  p->Day=*buf++;                //Day (1..31)
  p->Hour=*buf++;               //Hour (0..23)
  p->Min=*buf++;                //Min (0..59)
  p->Sec=*buf++;                //Sec (0..59)
  p->Valid=*buf++;              //Validity
                                //  2: Valid UTC (leap secs known)
                                //  1: GPS Week number valid
                                //  0: GPS Time of week valid

  //Mark message as having been updated.
  p->updated=1;
  }

//===================================================================
//                       StoreTIM_ERR
//===================================================================
//Store TIM_ERR message in the supplied structure mMIDG2_TIM_ERR
static void StoreTIM_ERR(mtMIDG2_TIM_ERR *p, unsigned char *buf)
  {
  //skip the length as the TIM_ERR message is always 10 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);   //GPS Milisecond Time of Week (ms)
  p->TTB=*buf++;                //Time timer bias (clocks)
  p->DTB=*buf++;                //Data timer bias (clocks)
  p->Flags=*buf++;              //Flags
                                //  6: Timestamp is GPS time
  //Mark message as having been updated.
  p->updated=1;
  }


//===================================================================
//                       StoreTIM_PPS
//===================================================================
//Store TIM_PPS message in the supplied structure mMIDG2_TIM_PPS
static void StoreTIM_PPS(mtMIDG2_TIM_PPS *p, unsigned char *buf)
  {
  //skip the length as the TIM_PPS message is always 16 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);   //GPS time (ms) of next pulse
  p->Frac=mBinGetULong(&buf);   //Fractional time (ms/2^32) of next pulse
  p->QErr=mBinGetLong(&buf);    //Quantization error (ps)
  p->Week=mBinGetUShort(&buf);  //GPS week of next pulse
  p->Flags=*buf++;              //Flags
                                //  0:  Time base is (0=GPS, 1=UTC)
                                //  1:  UTC is available
  p->res=*buf++;                //(reserved)

  //Mark message as having been updated.
  p->updated=1;
  }

//===================================================================
//                       StoreTIM_TM
//===================================================================
//Store TIM_TM message in the supplied structure mMIDG2_TIM_TM
static void StoreTIM_TM(mtMIDG2_TIM_TM *p, unsigned char *buf)
  {
  //skip the length as the TIM_TM message is always 8 bytes.
  ++buf;

  p->Time=mBinGetULong(&buf);   //GPS time (ms) of next pulse
  p->Week=mBinGetUShort(&buf);  //GPS week of next pulse
  p->res =mBinGetUShort(&buf);  //(reserved)

  //Mark message as having been updated.
  p->updated=1;
  }


