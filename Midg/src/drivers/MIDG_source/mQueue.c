/*
mQueue.c
October 18, 2005
Chad Phillips
*/
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------

#include <stdlib.h>
#include <malloc.h>
#include "mQueue.h"

//======================================================================
//                     Queue Definitions
//======================================================================
//Of specific concern are the read and write handle definitions.
//There is only one write handle for each queue.  All read handles reference
//it.  So, the write handle will hold the info about start and end of buffer
//as well as queue size.
struct mQueueWHs
  {
  void (*Callback)(void);       //call on write (zero=no call)
  unsigned char *Ptr;           //pointer into the queue
  unsigned char *BufPtr;        //start of queue
  unsigned char *EndPtr;        //byte after end of queue
  };

//There may be many read handles.  Each one will reference the queues write
//handle so that reads using the read handle can be checked to see if the
//queue is empty (which occurs when the read handle has caught up with the
//write handle).
struct mQueueRHs
  {
  mQueueWH   hw;          //write handle
  unsigned char *Ptr;     //pointer into the queue
  void (*Callback)(void); //call on read (zero=no call)
  };

//This stucture is provided to allow initial allocation of the packet queue
//with a single call to malloc.
typedef struct
  {
  struct mQueueRHs hr;
  struct mQueueWHs hw;
  char   buf_start;      //first byte of buffer
  } queueblk;

//======================================================================
//                         mQueueCreate
//======================================================================
//mQueueCreate creates a queue of the specified size and returns
//read and write handles for the queue in the supplied pointers.
//Returns zero on success.
int mQueueCreate(mQueueRH *QRH, mQueueWH *QWH, int mQSize)
  {
  //Space will be allocated in one shot with the read structure followed
  //by the write structure followed by the queue space.
  queueblk *p = malloc(sizeof(queueblk) + mQSize);
  if(!p)
    return(1);

  //The queue memory has been allocated, so lets initialize the fields.
  p->hw.Callback = 0;
  p->hw.Ptr    = &(p->buf_start);
  p->hw.BufPtr = &(p->buf_start);
  p->hw.EndPtr = &(p->buf_start) + mQSize;

  p->hr.hw  = &(p->hw);
  p->hr.Ptr = &(p->buf_start);
  p->hr.Callback = 0;

  //Finally, if the user provided handles, stuff the address of the new
  //structure into the handles.
  if(QRH)
    *QRH = &(p->hr);
  if(QWH)
    *QWH = &(p->hw);

  return(0);
  }


//======================================================================
//                       mQueueDupRH
//======================================================================
//Creates a duplicate read handle.  This read handle is initialized to 
//the same location in the stream as the source handle.  Callback
//functions are not duplicated, and the new handle will have no callback
//installed.  Returns zero on fail.
mQueueRH mQueueDupRH(mQueueRH rh)
  {
  mQueueRH dup = (mQueueRH)malloc(sizeof(struct mQueueRHs));
  if(dup)
    {
    dup->hw = rh->hw;
    dup->Ptr = rh->Ptr;
    dup->Callback = 0;
    }
  return(dup);
  }
  
//======================================================================
//                       mQueueDestroy
//======================================================================
//mQueueDestroy deallocates the memory allocated by the initial creation of
//a queue.  It may be called to destroy either the initial queue or duplicate
//handles.  Duplicate handles should be destroyed before the original queue.
void mQueueDestroy(mQueueRH hr)
  {
  free(hr);
  }

//======================================================================
//                       mQueueWriteNotify
//======================================================================
//To install a write notification funtion onto a queue, use the following:
void mQueueWriteNotify(mQueueWH hw, mQueueNotify f)
  {
  hw->Callback = f;
  }


//======================================================================
//                       mQueueReadNotify
//======================================================================
//To install a read notification funtion onto a queue, use the following:
void mQueueReadNotify(mQueueRH hr, mQueueNotify f)
  {
  hr->Callback = f;
  }


//======================================================================
//                         mQueueWriteChar
//======================================================================
//mQueueWriteChar places a single character into a queue.
void mQueueWriteChar(mQueueWH hw, char ch)
  {
  unsigned char *p = hw->Ptr;
  unsigned char *pend = hw->EndPtr;
  unsigned char *pbuf = hw->BufPtr;
  *p++ = ch;
  if(p >= pend)
    p = pbuf;
  hw->Ptr = p;
  if(hw->Callback)
    hw->Callback();
  }
//======================================================================
//                       mQueueWriteString
//======================================================================
//mQueueWriteString places 'n' characters from 'source' into the queue.
void mQueueWriteString(mQueueWH hw, char *source, short n)
  {
  int i;
  unsigned char *p = hw->Ptr;
  unsigned char *pend = hw->EndPtr;
  unsigned char *pbuf = hw->BufPtr;
  for(i=0;i<n;++i)
    {
    *p++ = *source++;
    if(p >= pend)
      p = pbuf;
    }
  hw->Ptr = p;
  if(hw->Callback)
    hw->Callback();
  }

//======================================================================
//                         mQueueReadChar
//======================================================================
//mQueueReadChar reads a single character from the queue.  If the queue is
//empty, the return value is -1.  Otherwise, the character is returned in the
//least significant byte of the short.
int mQueueReadChar(mQueueRH hr)
  {
  int i=-1;
  mQueueWH hw = hr->hw;
  unsigned char *r = hr->Ptr;
  unsigned char *w = hw->Ptr;
  unsigned char *pend = hw->EndPtr;
  unsigned char *pbuf = hw->BufPtr;
  if(r != w)
    {
    i = *r++;
    if(r >= pend)
      r = pbuf;
    }
  hr->Ptr = r;
  if(hr->Callback)
    hr->Callback();
  return(i);
  }
  
  
//======================================================================
//                         mQueueReadString
//======================================================================
//mQueueReadString reads upto 'n' characters from the queue and places them
//in the array at 'dest'.  The function returns the number of characters
//actually transferred.  Zero means that the queue is empty.
int mQueueReadString(mQueueRH hr, char *dest, short n)
  {
  int i;
  mQueueWH hw = hr->hw;
  unsigned char *r = hr->Ptr;
  unsigned char *w = hw->Ptr;
  unsigned char *pend = hw->EndPtr;
  unsigned char *pbuf = hw->BufPtr;
  for(i=0;(r!=w) && (i<n);++i)
    {
    *dest++ = *r++;
    if(r >= pend)
      r = pbuf;
    }
  hr->Ptr = r;
  if(hr->Callback)
    hr->Callback();
  return(i);
  }
