//mQueue.h
//October 18, 2005
//Chad Phillips
//Microbotics, Inc.
//--------------------------------------------------------------------------
//Copyright © 1999-2006 Microbotics, Inc., Hampton, VA. All rights reserved.
//http://microboticsinc.com
//Information contained in this document is proprietary to Microbotics, Inc.
//Any use or distribution without written permission is prohibited.
//--------------------------------------------------------------------------
/*
mQueue module.  Byte queues implemented by this module are used by 
serial communcation devices to exchange data with the user program.
*/

#ifndef _mQueue_Header_
  #define _mQueue_Header_
#if __cplusplus
extern "C" {
#endif

//Define handles are pointer to queue structures.
typedef struct mQueueRHs *mQueueRH;
typedef struct mQueueWHs *mQueueWH;

//Notification functions may be installed onto a queue as callback functions
//that are called upon read from or write to a queue.  This is most useful for
//transmit devices that need to have their interrupt enabled when new data
//is available to be transmitted.
typedef void (*mQueueNotify)(void);

//mQueueCreate creates a queue of the specified size and returns
//read and write handles for the queue in the supplied pointers.
//Returns zero on success.
int mQueueCreate(mQueueRH *, mQueueWH *, int mQSize);

//mQueueDupRH creates a duplicate read handle.  This read handle is
//initialized to the same location in the stream as the source handle.
//Callback functions are not duplicated, and the new handle will have no
//callback installed.  Returns zero on fail.
mQueueRH mQueueDupRH(mQueueRH rh);

//mQueueDestroy deallocates the memory allocated by the initial creation of
//a queue.  It may be called to destroy either the initial queue or duplicate
//handles.  Duplicate handles should be destroyed before the original queue.
void mQueueDestroy(mQueueRH hr);

//To install notification funtions onto a queue, use the following:
void mQueueWriteNotify(mQueueWH, mQueueNotify);
void mQueueReadNotify(mQueueRH, mQueueNotify);

//Write functions:
//mQueueWriteChar places a single character into a queue.
//mQueueWriteString places 'n' characters from 'source' into the queue.
void mQueueWriteChar(mQueueWH, char ch);
void mQueueWriteString(mQueueWH, char *source, short n);

//Read functions:
//mQueueReadChar reads a single character from the queue.  If the queue is
//empty, the return value is -1.  Otherwise, the character is returned in the
//least significant byte of the short.
//mQueueReadString reads upto 'n' characters from the queue and places them
//in the array at 'dest'.  The function returns the number of characters
//actually transferred.  Zero means that the queue is empty.
int mQueueReadChar(mQueueRH);
int mQueueReadString(mQueueRH, char *dest, short n);


#if __cplusplus
}
#endif
#endif
