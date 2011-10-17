#include <stdio.h>
#include <math.h>
#include "mMIDG2.h"
#include "rotate.h"
#include "ecef.h"

//Magnetic declination
#define MAG_DEC 0

//Local buffer for generating the messages
static unsigned char tbuf[256];

//SendNMEA calculates checksum and sends message
static void SendNMEA(mQueueWH hwOut, unsigned char *p);

//Get ASCII Hexidecimal value of a character
unsigned char *mHexFromCharZ(unsigned char *p, unsigned char c);

//==========================================================================
//                       NMEAOutput
//==========================================================================
//Create NMEA messages from MIDG II state information.
void NMEAOutput(mQueueWH hwOut, mtMIDG2State *M2)
  {
  int i;
  float vel[3];
  long int pos[3];
  double lat, lon, alt;
  unsigned char *p;

  //Get latitude, longitude, altitude
  i = ((M2->NAV_PV.Details)>>2)&0x3;      //get position format
  if(i == 0)
    {
    //Convert ECEF to LLA for output.
    ECEF2LLA(M2->NAV_PV.Pos[0]*0.01L,
             M2->NAV_PV.Pos[1]*0.01L,
             M2->NAV_PV.Pos[2]*0.01L,
             &lat, &lon, &alt);
    lat *= R2DL;
    lon *= R2DL;
    }
  else if(i == 1)
    return;                               //won't deal with ENU position for now
  else
    {
    //Convert LLA scaled integers
    lon = M2->NAV_PV.Pos[0]*1e-7;         //degrees
    lat = M2->NAV_PV.Pos[1]*1e-7;         //degrees
    alt = M2->NAV_PV.Pos[2]*1e-2;         //meters
    }


  //Now, get velocity in NED
  i = (M2->NAV_PV.Details)&0x2;           //get velocity format (ENU?)
  if(i)
    {
    //Velocity is ENU.  Convert to NED.
    vel[0] =  M2->NAV_PV.Vel[1]*0.01;
    vel[1] =  M2->NAV_PV.Vel[0]*0.01;
    vel[2] = -M2->NAV_PV.Vel[2]*0.01;
    }
  else
    {
    //velocity is ECEF, convert to NED.
    float Vecef[3], Vned[3], ypr[3];
    float q_ned2ecef[4], q_ecef2ned[4], q_ecef2uen[4];
    static float q_uen2ned[4] = {0.70710678118655, 0.0, -0.70710678118655, 0.0};

    /* get the ECEF to NED & NED to ECEF transformations */
    ypr[0] = lon*D2RL;
    ypr[1] = -lat*D2RL;
    ypr[2] = 0.0;
    CreateQ_YPR(q_ecef2uen, ypr);
    MultQ_Q(q_ecef2ned, q_ecef2uen, q_uen2ned);
    TransposeQ2(q_ned2ecef, q_ecef2ned);

    Vecef[0] = M2->NAV_PV.Vel[0]*0.01;
    Vecef[1] = M2->NAV_PV.Vel[1]*0.01;
    Vecef[2] = M2->NAV_PV.Vel[2]*0.01;
    MultQ_Vc(vel, q_ned2ecef, Vecef);
    }

  //======================= GGA ========================
	// Format GGA message without leading $ and trailing * for checksum calculations
	  {
    char lat_dir, lon_dir;
    int lat_deg, lon_deg;
    float lat_min, lon_min;
    int gps_hour, gps_min;
    float gps_sec;
    //int gps_day;

	  if ( lat >= 0.0 )
		  lat_dir = 'N';
	  else
		  lat_dir = 'S';
	  if ( lon >= 0.0 )
		  lon_dir = 'E';
	  else
		  lon_dir = 'W';

	  lat_deg = (int)fabs(lat);
	  lat_min = fabs(fmod(lat,1.0))*60;

	  lon_deg = (int)fabs(lon);
	  lon_min = fabs(fmod(lon,1.0))*60;

    //gps_day =  M2->NAV_PV.Time / (3600 * 24 * 1000);
    gps_hour = (M2->NAV_PV.Time % (3600 * 24 * 1000)) / (3600 * 1000);
    gps_min = (M2->NAV_PV.Time % (3600 * 1000))/ (60*1000);
    gps_sec = fmod(M2->NAV_PV.Time,60*1000)/1000;

    p=tbuf;
    p+=sprintf(p,"$GPGGA,%02d%02d%02.0f.0,", gps_hour, gps_min, gps_sec);
    p+=sprintf(p,"%02d", lat_deg);
    p+=sprintf(p,"%06.3f,", lat_min);
    p+=sprintf(p,"%c,%03d%06.3f,", lat_dir, lon_deg, lon_min);
       sprintf(p,"%c,1,%d,1.61,%05.0f,M,002,M,,",
               lon_dir,
               (M2->NAV_PV.Details >> 12) & 0xf,     //nsats
               alt);

    SendNMEA(hwOut, tbuf);
    }

  //======================= VTG ========================
	// Calculate track angle for VTG message
	  {
	  float cog, sog, mcog;

	  cog = atan2(vel[1],vel[0])*R2D;                //cog from Vned
	  if(cog < 0) cog += 360.0;

	  mcog = cog - MAG_DEC;                        //magnetic course?
	  if(mcog < 0) mcog += 360.0;
	  else if(mcog > 360.0) mcog -= 360.0;

	  sog = sqrt(vel[0]*vel[0] + vel[1]*vel[1]);     //sog from Vned

	  // Format VTG message without leading $ and trailing * for checksum calculations
	  p=tbuf;
	  p+=sprintf(p,"$GPVTG,%05.1f,T,%05.1f,M,", cog, mcog);
	     sprintf(p,"%06.2f,N,%06.2f,K",	sog * 1.9438445, sog * 3.6 );

    SendNMEA(hwOut, tbuf);
	  }
  }


//==========================================================================
//                       SendNMEA
//==========================================================================
//SendNMEA expects a null terminated NMEA string to which it must append the
//correct checksum and send out the specified serial stream.
static void SendNMEA(mQueueWH hwOut, unsigned char *p)
  {
  //Take a NMEA string, calculate and append it's checksum, and send it
  //out the NMEA port.
  unsigned char *q;
  unsigned char cksum=0;

  //Do checksum as xor of all bytes from the first char after the opening
  //'$' to the end of the string.  The provided string is assumed to start
  //with '$' and be null terminated without the checksum start character '*'.
  for(q=p+1; *q != 0; ++q)
    cksum ^= *q;
  *q++ = '*';
  q=mHexFromCharZ(q, cksum);     //keeps leading zeros
  *q++ = 0xD;
  *q++ = 0xA;

  //Write NMEA out to the stream output
  mQueueWriteString(hwOut, tbuf, q-p);
  }



//==========================================================================
//                       mHexFromCharZ
//==========================================================================
//Generate an ascii representation if a character in hexadecimal.
static unsigned char hex_table[] = {
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

unsigned char *mHexFromCharZ(unsigned char *p, unsigned char c)
  {
  *p++ = hex_table[c>>4];
  *p++ = hex_table[c&0xF];
  return(p);
  }

