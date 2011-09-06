#include <iostream.h>
#include <stdlib.h>
#include "comport.h"


ComPort::ComPort()
  {
  State = CPS_UnInit;
  SelectedBaud = 115200;
  }

ComPort::~ComPort()
  {
  if(State == CPS_Open)
    Close();
  if(State == CPS_Closed)
    UnInit();
  }

int ComPort::Init(char *Name)
  {
  if(State != CPS_UnInit)
    return(1);

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);
  try
    {
    //create the read and write events for overlapped IO
    ReadOvl.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    WriteOvl.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    if(ReadOvl.hEvent == INVALID_HANDLE_VALUE ||
       WriteOvl.hEvent == INVALID_HANDLE_VALUE)
      throw 2;
    }
  catch(int error)
    {
    if(ReadOvl.hEvent != INVALID_HANDLE_VALUE)
      {
      CloseHandle(ReadOvl.hEvent);
      ReadOvl.hEvent = INVALID_HANDLE_VALUE;
      }
    if(WriteOvl.hEvent != INVALID_HANDLE_VALUE)
      {
      CloseHandle(WriteOvl.hEvent);
      WriteOvl.hEvent = INVALID_HANDLE_VALUE;
      }
    delete Lock;
    return(error);
    }

  //initialization (that is unlikely to fail)
  BytesRxd=BytesTxd=0;
  PortName = std::string(Name);
  PortHandle = INVALID_HANDLE_VALUE;
  State = CPS_Closed;
  delete Lock;
  return(0);
  }

int ComPort::UnInit()
  {
  if(State == CPS_UnInit)
    return(0);
    
  if(State == CPS_Open)
    Close();

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);
  if(ReadOvl.hEvent != INVALID_HANDLE_VALUE)
    CloseHandle(ReadOvl.hEvent);
  if(WriteOvl.hEvent != INVALID_HANDLE_VALUE)
    CloseHandle(WriteOvl.hEvent);
  ReadOvl.hEvent = WriteOvl.hEvent = INVALID_HANDLE_VALUE;
  State = CPS_UnInit;
  delete Lock;
  return(0);
  }

int ComPort::Open()
  {
  //check to make sure the port hasn't already been opened or is uninitialized
  if(State != CPS_Closed)
    return(1);

  DCB dcb;
  COMMTIMEOUTS TimeOuts;

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);
  try
    {
    //open the write handle
    PortHandle = CreateFile(PortName.c_str(), GENERIC_READ|GENERIC_WRITE,
                            0, /* exclusive access - not shared */
                            NULL, /* no security attrs */
                            OPEN_EXISTING,
                            FILE_FLAG_OVERLAPPED,
                            NULL);

    if(PortHandle == INVALID_HANDLE_VALUE)
      throw 2;

    /* get current device control block info */
    fSuccess = GetCommState(PortHandle, &dcb);

    if(!fSuccess)
      throw 3;

    /* set device control block info */
    dcb.fBinary = TRUE;
    dcb.fNull = FALSE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    fSuccess = SetCommState(PortHandle, &dcb);

    if(!fSuccess)
      throw 4;

    TimeOuts.ReadIntervalTimeout = 0;
    TimeOuts.ReadTotalTimeoutMultiplier = 0;
    TimeOuts.ReadTotalTimeoutConstant = 0;  //200
    TimeOuts.WriteTotalTimeoutMultiplier = 0;
    TimeOuts.WriteTotalTimeoutConstant = 0;

    fSuccess = SetCommTimeouts(PortHandle, &TimeOuts);
    if(!fSuccess)
      throw 5;

    fSuccess = PurgeComm(PortHandle, PURGE_TXABORT | PURGE_RXABORT |
                                     PURGE_TXCLEAR | PURGE_RXCLEAR);
    if(!fSuccess)
      throw 6;
    }
  catch(int error)
    {
    if(PortHandle != INVALID_HANDLE_VALUE)
      {
      CloseHandle(PortHandle);
      PortHandle = INVALID_HANDLE_VALUE;
      }
    delete Lock;
    return(error);
    }

  //reset the byte counters and update the state
  BytesRxd=BytesTxd=0;
  State = CPS_Open;
  OutputDebugString("ComPort::Open successful.");
  delete Lock;

  SetBaudRate(SelectedBaud);
  
  return(0);
  }

int ComPort::Close()
  {
  //if the port is not open, there is nothing to do.
  if(State != CPS_Open)
    return(0);

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);

  if(PortHandle != INVALID_HANDLE_VALUE)
    {
    PurgeComm(PortHandle, PURGE_TXABORT | PURGE_RXABORT |
                          PURGE_TXCLEAR | PURGE_RXCLEAR);
    CloseHandle(PortHandle);
    PortHandle = INVALID_HANDLE_VALUE;
    }
  State = CPS_Closed;
  delete Lock;
  return(0);
  }

void ComPort::SetLine(int databits, char parity, int stopbits)
  {
  DCB dcb;

  if(State != CPS_Open)
    return;

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);

  fSuccess = GetCommState(PortHandle, &dcb);

  //validate the number of bits specified
  if(databits != 8 && databits != 7 && databits != 5)
    {
    cout<<"Invalid parameter databits in ComPort::SetLine, port "
        << PortName <<endl;
    exit(1);
    }
  dcb.ByteSize = databits;

  //validate the specified parity
  switch(parity)
    {
    case 'n':
    case 'N': dcb.Parity = NOPARITY;
              break;
    case 'o':
    case 'O': dcb.Parity = ODDPARITY;
              break;
    case 'e':
    case 'E': dcb.Parity = EVENPARITY;
              break;
    case 'm':
    case 'M': dcb.Parity = MARKPARITY;
              break;

    default: cout<<"Invalid parity in ComPort::SetLine, port "
                 <<PortName <<endl;
             exit(1);
    };

  //validate the number of stop bits
  //in this function: 1 = 1 bit, 2 = 2 bits,   3 = 1.5 bits
  //in the API call : 0 = 1 bit, 1 = 1.5 bits, 2 = 2 bits
  if(stopbits == 1)
    dcb.StopBits = ONESTOPBIT;
  else if(stopbits == 2)
    dcb.StopBits = TWOSTOPBITS;
  else if(stopbits == 3)
    dcb.StopBits = ONE5STOPBITS;
  else
    {
    cout<<"Invalid parameter stopbits in ComPort::SetLine, port "
        << PortName <<endl;
    exit(1);
    }

  fSuccess = SetCommState(PortHandle, &dcb);

  if (!fSuccess)
    {
    cout<<"Cannot set line settings on com port " <<PortName <<endl;
    exit(1);
    }
  delete Lock;
  }

void ComPort::SetBaudRate(unsigned int baudrate)
  {
  DCB dcb;

  if(State != CPS_Open)
    {
    SelectedBaud = baudrate;  //will get set on next open.
    return;
    }

  TCOMCriticalSection::Lock *Lock = new TCOMCriticalSection::Lock(CritSec);

  fSuccess = GetCommState(PortHandle, &dcb);

  switch(baudrate)
    {
    case 300:
      dcb.BaudRate = CBR_300;
      break;
    case 1200:
      dcb.BaudRate = CBR_1200;
      break;
    case 2400:
      dcb.BaudRate = CBR_2400;
      break;
    case 4800:
      dcb.BaudRate = CBR_4800;
      break;
    case 9600:
      dcb.BaudRate = CBR_9600;
      break;
    case 19200:
      dcb.BaudRate = CBR_19200;
      break;
    case 38400:
      dcb.BaudRate = CBR_38400;
      break;
    case 57600:
      dcb.BaudRate = CBR_57600;
      break;
    case 115200:
      dcb.BaudRate = CBR_115200;
      break;
    default:
      dcb.BaudRate = CBR_9600;
    };

  fSuccess = SetCommState(PortHandle, &dcb);

  if (!fSuccess)
    {
    cout<<"Cannot set baud rate on com port " <<PortName <<endl;
    exit(1);
    }
  delete Lock;

  SelectedBaud = baudrate;
  }

int ComPort::GetChar(char *ch)
  {
  if(State != CPS_Open)
    return(1);

  ULONG BytesRead;
  DWORD error;

  //ReadFile should block until a character is received
  fSuccess = ReadFile(PortHandle, (LPVOID) ch, 1L, &BytesRead, &ReadOvl);

  if(!fSuccess)
    {
    error = GetLastError();
    if(error == ERROR_IO_PENDING)
      {
      GetOverlappedResult(PortHandle, &ReadOvl, &BytesRead, TRUE);
      if(BytesRead > 1)
        {
        char buf[100];
        sprintf(buf,"ComPort::GetChar read %d byte(s).",BytesRead);
        OutputDebugString(buf);
        }
      if(BytesRead == 0)
        return(1);
      }
    else
      {
      char buf[100];
      sprintf(buf,"ComPort::GetChar error %d.",error);
      OutputDebugString(buf);
      ClearCommError(PortHandle, &error, NULL);
      sprintf(buf,"ComPort::GetChar cleared comm error %d.",error);
      OutputDebugString(buf);
      return(1);
      }
    }

  BytesRxd += BytesRead;
  return(0);
  }

int ComPort::GetNChars(int n, char *ch)
  {
  if(State != CPS_Open)
    return(-1);

  ULONG BytesRead;
  DWORD error;

  //ReadFile should block until a character is received
  fSuccess = ReadFile(PortHandle, (LPVOID) ch, n, &BytesRead, &ReadOvl);

  if(!fSuccess)
    {
    error = GetLastError();
    if(error == ERROR_IO_PENDING)
      {
      GetOverlappedResult(PortHandle, &ReadOvl, &BytesRead, TRUE);
      if(BytesRead <= 0UL)
        return(-1);  //must have timed out
      }
    else
      {
      ClearCommError(PortHandle, &error, NULL);
      return(-1);
      }
    }

  BytesRxd += BytesRead;
  return(BytesRead);
  }

void ComPort::PutChar(char c)
  {
  if(State != CPS_Open)
    return;

  char temp=c;
  ULONG Written;
  DWORD error;

  fSuccess = WriteFile(PortHandle, (LPCVOID)&temp, sizeof(char),
                       &Written, &WriteOvl);

  if(!fSuccess)
    {
    error = GetLastError();
    if(error == ERROR_IO_PENDING)
      GetOverlappedResult(PortHandle, &WriteOvl, &Written, TRUE);
    else
      {
      ClearCommError(PortHandle, &error, NULL);
      return;
      }
    }

  BytesTxd += Written;
  }

void ComPort::PutArray(char *buf, int n)
  {
  if(State != CPS_Open)
    return;

  ULONG Written;
  DWORD error;

  fSuccess = WriteFile(PortHandle, (LPCVOID)buf, n, &Written, &WriteOvl);

  if(!fSuccess)
    {
    error = GetLastError();
    if(error == ERROR_IO_PENDING)
      GetOverlappedResult(PortHandle, &WriteOvl, &Written, TRUE);
    else
      {
      char buf[100];
      sprintf(buf,"ComPort::PutArray error %d.",error);
      OutputDebugString(buf);
      ClearCommError(PortHandle, &error, NULL);
      sprintf(buf,"ComPort::PutArray cleared comm error %d.",error);
      OutputDebugString(buf);
      return;
      }
    }

  BytesTxd += Written;
  }


