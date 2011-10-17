//mBin.c
//Chad Phillips
//Microbotics, Inc.
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------

//This module provides the routines needed to work with the Microbotics binary
//protocol.  Routines are provided to construct packets as well as receive and
//validate packets.

#include <malloc.h>
#include "mBin.h"

//==========================================================================
//                         mBinPacketOpen
//==========================================================================
//Start a mBin packet in 'buffer' with the given ID.  A pointer to the byte
//location after the ID is returned.
unsigned char *mBinPacketOpen(unsigned char *buffer, unsigned char ID)
  {
  //Add the packet start word and the packet ID.  Also reserve space for the
  //packet byte count that will be written by the mBinPacketClose function.
  *buffer++ = 0x81;             //packet head
  *buffer++ = 0xA1;
  *buffer++ = ID;               //packet ID
  *buffer++ = 0x0;              //space reserved for the byte count
  return(buffer);
  }

//==========================================================================
//                         mBinPacketClose
//==========================================================================
//Close a packet that has been started with mBinPacketOpen and optionally
//filled with data.  This routine calculates and appends a checksum.  It
//returns the total packet length constructed into 'buffer'.
unsigned short mBinPacketClose(unsigned char *buffer, unsigned char *p_end)
  {
  unsigned char *p;
  unsigned char pay_len;
  unsigned char ck0=0;
  unsigned char ck1=0;

  //First, lets calculate the number of payload bytes in the buffer and
  //write the payload byte count into the packet.
  pay_len = p_end - buffer - 4;
  buffer[3] = pay_len;

  //Now, calculate the fletcher checksum of the ID-Count bytes and all of
  //the payload bytes.
  p=buffer+2;
  while(p < p_end)              //add the payload words
    {
    ck0 += *p++;
    ck1 += ck0;
    }

  //Append the checksum to the packet.
  *p++ = ck0;
  *p++ = ck1;

  //Finally, we would like to return the number of bytes in the packet.
  return(p-buffer);
  }

//==========================================================================
//                         mBinPutShort
//==========================================================================
//Appends a short integer to the mBin packet.
unsigned char *mBinPutShort(unsigned char *p, short int h)
  {
  return(mBinPutUShort(p,(unsigned short)h));
  }

//==========================================================================
//                         mBinPutUShort
//==========================================================================
//Appends an unsigned  short integer to the mBin packet.
unsigned char *mBinPutUShort(unsigned char *p, unsigned short uh)
  {
  unsigned char uc0, uc1;  //uc0 is LSB, uc1 is MSB

  uc0 = uh&0xFF;
  uh >>= 8;
  uc1 = uh&0xFF;

  *p++=uc1;
  *p++=uc0;

  return(p);
  }

//==========================================================================
//                         mBinPutLong
//==========================================================================
//Appends a long integer to the mBin packet.
unsigned char *mBinPutLong(unsigned char *p, long int d)
  {
  return(mBinPutULong(p,(unsigned long)d));
  }

//==========================================================================
//                         mBinPutULong
//==========================================================================
//Appends an unsigned long integer to the mBin packet.
unsigned char *mBinPutULong(unsigned char *p, unsigned long ud)
  {
  unsigned char uc0, uc1, uc2, uc3;  //uc0 is LSB, uc3 is MSB

  uc0 = ud&0xFF;
  ud >>= 8;
  uc1 = ud&0xFF;
  ud >>= 8;
  uc2 = ud&0xFF;
  ud >>= 8;
  uc3 = ud&0xFF;

  *p++=uc3;
  *p++=uc2;
  *p++=uc1;
  *p++=uc0;

  return(p);
  }


//==========================================================================
//                         mBinPutFloat
//==========================================================================
//Appends a float to the mBin packet.
unsigned char *mBinPutFloat(unsigned char *p, float f)
  {
  unsigned char uc0, uc1, uc2, uc3;  //uc0 is LSB, uc3 is MSB
  unsigned long ud = *((unsigned long *)&f);

  uc0 = ud&0xFF;
  ud >>= 8;
  uc1 = ud&0xFF;
  ud >>= 8;
  uc2 = ud&0xFF;
  ud >>= 8;
  uc3 = ud&0xFF;

  *p++=uc3;
  *p++=uc2;
  *p++=uc1;
  *p++=uc0;

  return(p);
  }


//==========================================================================
//                         mBinPutDouble
//==========================================================================
//Appends a double to the mBin packet.
unsigned char *mBinPutDouble(unsigned char *p, double lf)
  {
  unsigned char *q = (unsigned char *)&lf;
  #ifdef _M_IX86
  *p++ = q[7];
  *p++ = q[6];
  *p++ = q[5];
  *p++ = q[4];
  *p++ = q[3];
  *p++ = q[2];
  *p++ = q[1];
  *p++ = q[0];
  #else
  *p++ = q[0];
  *p++ = q[1];
  *p++ = q[2];
  *p++ = q[3];
  *p++ = q[4];
  *p++ = q[5];
  *p++ = q[6];
  *p++ = q[7];
  #endif
  return(p);
  }

//==========================================================================
//                         mBinGetShort
//==========================================================================
//Retrieves a short integer from the buffer *p and updates the pointer to
//point to the next data item.
short int mBinGetShort(unsigned char **p)
  {
  short int h;
  unsigned char *q = *p;
  h = *q++;
  h = (h<<8)|(*q++);
  *p = q;
  return(h);
  }

//==========================================================================
//                         mBinGetUShort
//==========================================================================
//Retrieves an unsigned short integer from the buffer *p and updates the
//pointer to point to the next data item.
unsigned short mBinGetUShort(unsigned char **p)
  {
  unsigned short uh;
  unsigned char *q = *p;
  uh = *q++;
  uh = (uh<<8)|(*q++);
  *p = q;
  return(uh);
  }

//==========================================================================
//                         mBinGetLong
//==========================================================================
//Retrieves a long integer from the buffer *p and updates the
//pointer to point to the next data item.
long int mBinGetLong(unsigned char **p)
  {
  long int d;
  unsigned char *q = *p;
  d = *q++;
  d = (d<<8)|(*q++);
  d = (d<<8)|(*q++);
  d = (d<<8)|(*q++);
  *p = q;
  return(d);
  }

//==========================================================================
//                         mBinGetULong
//==========================================================================
//Retrieves an unsigned long integer from the buffer *p and updates the
//pointer to point to the next data item.
unsigned long mBinGetULong(unsigned char **p)
  {
  unsigned long ud;
  unsigned char *q = *p;
  ud = *q++;
  ud = (ud<<8)|(*q++);
  ud = (ud<<8)|(*q++);
  ud = (ud<<8)|(*q++);
  *p = q;
  return(ud);
  }


//==========================================================================
//                         mBinGetFloat
//==========================================================================
//Retrieves a float from the buffer *p and updates the
//pointer to point to the next data item.
float mBinGetFloat(unsigned char **p)
  {
  float f;
  unsigned long ud;
  unsigned char *q = *p;
  ud = *q++;
  ud = (ud<<8)|(*q++);
  ud = (ud<<8)|(*q++);
  ud = (ud<<8)|(*q++);
  *p = q;
  f = *((float *)&ud);
  return(f);
  }


//==========================================================================
//                         mBinGetDouble
//==========================================================================
//Retrieves a double from the buffer *p and updates the
//pointer to point to the next data item.
double mBinGetDouble(unsigned char **p)
  {
  union
    {
    unsigned char buf[8];
    double lf;
    } u;
  int i;
  unsigned char *q = *p;

  #ifdef _M_IX86
  unsigned char *b = u.buf+7;
  for(i=0;i<8;++i)
    *b-- = *q++;
  #else
  unsigned char *b = u.buf;
  for(i=0;i<8;++i)
    *b++ = *q++;
  #endif
  *p = q;
  return(u.lf);
  }


//==========================================================================
//                         mBinCreatePacketizer
//==========================================================================
mBinPacketizer mBinCreatePacketizer(mQueueRH rh, int max_pkt_size)
  {
  //Try to allocate the packetizer structure.
  mBinPacketizer p = (mBinPacketizer)malloc(sizeof(struct mBinPacketizerS));
  if(!p)
    return(0);          //return zero if unable to allocate.

  //Initialize the packetizer structure.
  p->qrh = rh;
  p->buf_size = max_pkt_size;
  p->state = 0;
  p->lbuf = (char *)malloc(max_pkt_size+4);
  if(p->lbuf == 0)
    {
    free(p);
    return(0);          //return zero if unable to allocate.
    }

  p->pLB = p->lbuf;
  p->count = 0;
  p->pkt_errors = 0;

  return(p);
  }


//==========================================================================
//                         mBinDeletePacketizer
//==========================================================================
void mBinDeletePacketizer(mBinPacketizer p)
  {
  if(p)
    {
    if(p->lbuf)
      {
      free(p->lbuf);
      p->lbuf = 0;
      }

    free(p);
    }
  }



//==========================================================================
//                         mBinGetPacket
//==========================================================================
//Retrieves bytes from the read queue and returns a pointer to a packet if
//one is received.  If the read queue has been depleted and no packets are
//retrieved, a null pointer is returned.  This function should be called until
//a null pointer is returned, and the user should handle each packet that is
//returned.
unsigned char *mBinGetPacket(mBinPacketizer p)
  {
  int ch;
  unsigned char uc;
  static unsigned char ck0, ck1;


  //Get a character from the queue and handle it based on our current
  //parser state.  The states we will implement are
  //    WaitStart0        wait for 0x81
  //    WaitStart1        wait for 0xA1
  //    WaitID            wait for ID character
  //    WaitCount         wait for the message byte count
  //    WaitPayload       wait for all of the payload characters.
  //    WaitCksum0        wait for the checksum char 0
  //    WaitCksum1        wait for checksum char 1
  while(1)
    {
    ch = mQueueReadChar(p->qrh);

    //if the queue is empty, return zero.
    if(ch == -1)
      return(0);

    //convert to unsigned character
    uc = ch;

    //we have a character, so lets deal with it.
    switch(p->state)
      {
      //WaitStart0 - 0x81
      case 0:
        if(uc == 0x81)
          {
          p->pLB = p->lbuf;     //initialize the current pointer
          *(p->pLB)++ = uc;     //store the 0x81
          ++(p->state);         //increment state
          }
        break;

      //WaitStart1 - 0xA1
      case 1:
        if(uc == 0xA1)
          {
          *(p->pLB)++ = uc;     //store the 0xA1
          ++(p->state);         //increment state
          }
        else
          p->state = 0;
        break;

      //WaitID
      case 2:
        //we just received an ID.  Store it and go.
        *(p->pLB)++ = uc;     //store the ID
        ++(p->state);         //increment state
        ck0 = uc;             //start the checksum;
        ck1 = uc;
        break;

      //WaitCount - payload byte count
      case 3:
        //Lets make sure that this count will not overflow our buffer.  That
        //would obviously be unacceptable.
        if(uc > (p->buf_size - 6))
          {
          ++(p->pkt_errors);
          p->state = 0;
          }
        else
          {
          //store the count in the buffer and our remaining word counter.
          *(p->pLB)++ = uc;
          p->count = uc;

          //update the checksum
          ck0 += uc;
          ck1 += ck0;

          //increment the state
          ++(p->state);
          //If payload length is zero, increment again to skip payload wait
          if(!uc)
            ++(p->state);
          }
        break;

      //WaitPayload - payload data
      case 4:
        //Wait for p->count payload bytes.  If we have received all of
        //the payload bytes, goto next state.
        *(p->pLB)++ = uc;
        ck0 += uc;              //update checksum
        ck1 += ck0;
        if(!(--(p->count)))
          ++(p->state);         //increment state
        break;

      //WaitCksum0 - MSByte of checksum
      case 5:
        if(uc != ck0)
          {
          ++(p->pkt_errors);
          p->state = 0;
          }
        else
          {
          *(p->pLB)++ = uc;     //store ck0
          ++(p->state);         //increment state
          }
        break;

      //WaitCksum1 - LSByte of cksum
      case 6:
        *(p->pLB)++ = uc;       //store ck1

        //regardless of outcome, the next state will be zero.
        p->state = 0;

        //Now for the test.
        if(uc != ck1)
          ++(p->pkt_errors);
        else
          return((p->lbuf) + 2);
        break;
      }
    }
  }

