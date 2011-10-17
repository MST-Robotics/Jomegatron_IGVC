//mBin.h
//Chad Phillips
//Microbotics, Inc.
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------

/*
This module provides the routines needed to work with the Microbotics binary
protocol.  Routines are provided to construct packets as well as receive and
validate packets.

The Microbotics binary protocol is a standard binary packet format that has
the following structure.  Each packet is composed of:

Element         characters      Content
Head            2               0x81A1
ID-Count        2               MSB=Message ID, LSB=number of payload bytes
Payload         n               Specific to the message type (ID).
Checksum        2               Two byte fletcher checksum.

The checksum is a fletcher checksum as defined in internet RFC 1145.  It is
computed over the byte between the head and checksum.  I.e., it includes the
ID and count bytes, and the payload bytes.  The basic algorithm is as follows.
        unsigned char byte0=0, byte1=0;
        unsigned short checksum;
        unsigned char *p = (address of message ID)
        for(i=0;i<payload_bytes+2;++i)
          {
          byte0 += *p++;
          byte1 += byte0;
          }
        checksum = ((byte0<<8) | byte1);

All of the words are transmitted MSB first.

*/

#ifndef _mBin_Header_
  #define _mBin_Header_

#include "mQueue.h"

#if __cplusplus
extern "C" {
#endif

//Routines for starting and finishing a packet in a buffer.
//mBinPacketOpen returns a pointer to the start of payload so that the mBinPut
//routines can be used to add data to the message.  The mBinPacketClose
//routine returns the length of the message stored in buffer.
unsigned char *mBinPacketOpen(unsigned char *buffer, unsigned char ID);
unsigned short mBinPacketClose(unsigned char *buffer, unsigned char *p_end);

//routines for adding data to a packet
unsigned char *mBinPutShort(unsigned char *p, short int h);
unsigned char *mBinPutUShort(unsigned char *p, unsigned short uh);
unsigned char *mBinPutLong(unsigned char *p, long int d);
unsigned char *mBinPutULong(unsigned char *p, unsigned long ud);
unsigned char *mBinPutFloat(unsigned char *p, float f);
unsigned char *mBinPutDouble(unsigned char *p, double lf);

//routines for retrieving a data from a packet.
short int       mBinGetShort(unsigned char **p);
unsigned short  mBinGetUShort(unsigned char **p);
long int        mBinGetLong(unsigned char **p);
unsigned long   mBinGetULong(unsigned char **p);
float           mBinGetFloat(unsigned char **p);
double          mBinGetDouble(unsigned char **p);



typedef struct mBinPacketizerS
  {
  mQueueRH      qrh;
  int           buf_size;
  int           state;
  unsigned long pkt_errors;
  unsigned char *lbuf;
  unsigned char *pLB;
  unsigned char count;
  } *mBinPacketizer;

//Functions to create and delete the packetizing mechanism.  The packetizing
//mechanism keeps up with the assigned serial queue and memory references so
//that the user need only call mGetBinPacket with a pointer to the mechanism
//stucture in order to get packets from a serial device.
mBinPacketizer mBinCreatePacketizer(mQueueRH, int max_pkt_size);
void mBinDeletePacketizer(mBinPacketizer);

//mBinGetPacket extracts bytes from the queue looking for packets.  It
//returns when either a packet has retrieved or the queue is empty.  If a
//valid packet is successfully retrieved from the queue, it is stored in the
//packetizer structure and a pointer to the packet is returned.  In this case,
//the pointer has been advanced two bytes so that it points to the message
//ID.  If the queue has been emptied in search of a complete valid packet,
//this routine will store information about any packet in process and return
//zero.
unsigned char *mBinGetPacket(mBinPacketizer);


#if __cplusplus
}
#endif
#endif
