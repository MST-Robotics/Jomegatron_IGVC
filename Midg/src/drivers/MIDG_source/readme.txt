readme.txt
May 26, 2005
Microbotics, Inc.

This directory contains example code for talking to the Microbotics MIDG II.
Modules are provided for handling the Microbotics Binary Protocol as well
as parsing the MIDG II output and storing the data in global structures.

This particular example (see main.c) reads MIDG II data from a file and looks
only for IMU_DATA messages and GPS_PV messages.  It produces a single output
line for every IMU_DATA sample retrieved from the MIDG II data file.  For each
sample, it appends the current GPS_PV data as well, preceeded by a flag
indicating when the GPS_PV data has been updated.

The syntax for the resulting executable is:
  program_name.exe <input file>

Output goes to C standard output, which is the console unless redirected.
To save the data to a file named out.txt, use somthing like:
  program_name infile.dat >out.txt

If the '#define NMEA' line is uncommented at the top of main.c, and appropriate
modules are included from the 'misc' subdirectory, the parser will generate
GGA and VTG NMEA messages from any NAV_PV messages in the input stream and
write them to the file 'nmea_out.txt'.


--------------------------------------------------------------------------
The misc subdirectory contains a comport module that can be used for serial
communication under windows.  When combined with the MIDG2 parsing module,
a real-time parser of MIDG II data can be created.


--------------------------------------------------------------------------
Creating MIDG II messages:
MIDG II command messages may be created using the mBin module.  As an example,
consider polling the TIM_UTC message.  Output messages are polled by sending
to the MIDG II a message with the same ID but with no payload.  The ID of
the TIM_UTC message is 25.

------------
unsigned char buf[40], *p;
unsigned short length;

p = mBinPacketOpen(buf, 25);      //25 is the ID of the message
length=mBinPacketClose(buf, p);
------------

At this point, the buffer 'buf' contains a valid MIDG II message of 'length'
bytes that can be sent to the MIDG II to request output of message 25.



As another example, consider providing heading measurements to the MIDG II.
For ground vehicles, the GPS velocity vector is often a valid measure of
heading when the vehicle is in motion.  In this example, velocity vector
from the GPS_PV message is used to provide heading measurements.
It is assumed that the velocity format is ENU, and the GPS_PV message has 
already been parsed and is held in M2->GPS_PV.

------------
float vel[3], magnitude;

//Get ENU velocity in m/s
vel[0] = M2->GPS_PV.Vel[0]*1e-2;
vel[1] = M2->GPS_PV.Vel[1]*1e-2;
vel[2] = M2->GPS_PV.Vel[2]*1e-2;

magnitude = sqrt(vel[0]*vel[0] + vel[1]*vel[1]);    //speed over ground

//If speed exceeds 1 m/s, make a heading measurement.
if(magnitude > 1.0)
  {
  unsigned char buf[40], *p;
  unsigned short length;
  short psi;                        //heading measurement (unit=0.1 deg)
  
  psi = atan2(vel[0],vel[1])*R2D*10;    //atan2(ve,vn)
  psi = (psi<<3)|2;                     //include confidence level

  //Create the heading measurement message (ID = 31).  This message includes 
  //a single short integer.  See the MIDG II message spec document.
  p = mBinPacketOpen(buf, 31);
  p = mBinPutShort(p, psi);
  length=mBinPacketClose(buf, p);

  //Send the message to the MIDG using the appropriate mechanism for your
  //platform.  It is assumed that a function SendToM2 exists to handle
  //the implementation details.  It takes a character pointer and the number
  //of characters to be sent as parameters.
  SendToM2(buf,length);
  }
------------


