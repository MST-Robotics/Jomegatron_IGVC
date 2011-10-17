#ifndef _ComPort_Header_
#define _ComPort_Header_
#include <wtypes.h>
#include <winbase.h>
#include <utilcls.h>
#include <string>

typedef enum
  {
  CPS_UnInit,
  CPS_Closed,
  CPS_Open
  } ComPortState;

class ComPort
  {
  private:
    std::string PortName;
    BOOL fSuccess;
    HANDLE PortHandle;
    OVERLAPPED ReadOvl;
    OVERLAPPED WriteOvl;
    ComPortState State;
    TCOMCriticalSection CritSec;
    unsigned int SelectedBaud;

  public:
    unsigned long int BytesRxd, BytesTxd;

    ComPort();
    ~ComPort();

    int Init(char *PortName);
    int UnInit();
    int Open();
    int Close();
    void SetLine(int databits, char parity, int stopbits);
    void SetBaudRate(unsigned int baudrate);

    ComPortState GetState() {return State;}

    int           GetChar(char *ch);
    int           GetNChars(int n, char *ch);
    void          PutChar(char c);
    void          PutArray(char *buf, int n);
  };
#endif
