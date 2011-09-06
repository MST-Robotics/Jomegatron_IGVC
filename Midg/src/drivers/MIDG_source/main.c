#include <stdio.h>
#include "mMIDG2.h"

//If the following define is uncommented, then NMEA data will be written to
//a file named nmea_out.txt.
//#define NMEA

#ifdef NMEA
extern void NMEAOutput(mQueueWH hwOut, mtMIDG2State *M2);
#endif

int main(int argc, char* argv[])
  {
  int i;
  char ch;
  unsigned long pkts=0;
  FILE *fp;
  mQueueRH hrMIDGrx;
  mQueueWH hwMIDGrx;
  mtMIDG2State *M2;
  
  #ifdef NMEA
  //Variables for NMEA output
  mQueueRH hrNMEA;
  mQueueWH hwNMEA;
  FILE *fpnmea;
  #endif
  
  //Check arguments
  if(argc != 2)
    {
    fprintf(stderr,"usage: %s <in_file>\n  output goes to stdout\n", argv[0]);
    return(1);
    }

  //Open the file
  fp = fopen(argv[1],"rb");
  if(!fp)
    {
    fprintf(stderr,"problem opening input file\n");
    return(1);
    }


  //create queues to be used by midg2 parser, and setup the parser
  mQueueCreate(&hrMIDGrx, &hwMIDGrx, 1024);   //create the midg rx queue
  M2=mMIDG2Setup(hrMIDGrx);

  #ifdef NMEA
  //create an output queue for NMEA messages that optionally get generated
  //from the MIDG messages
  mQueueCreate(&hrNMEA, &hwNMEA, 1024);
  
  //Open the NMEA output file.
  fpnmea = fopen("nmea_out.txt","w");
  if(!fpnmea)
    {
    fprintf(stderr,"problem opening NMEA output file\n");
    return(1);
    }
  #endif
  
  while((i=fgetc(fp)) != EOF)
    {
    unsigned char ID;
    
    ch=i;
    mQueueWriteChar(hwMIDGrx,ch);

    //While mMIDG2Parse returns non-zero, the return value is the ID.
    while(ID = mMIDG2Parse(M2))
      {
      //This example produces a single output line for every IMU_DATA sample
      //retrieved from the MIDG II data file.  For each sample, it appends
      //the current GPS_PV data as well, preceeded by a flag indicating when
      //the GPS_PV data has been updated.
      switch(ID)
        {
        case 2:   //IMU_DATA
          {
          fprintf(stderr,"%lu\r",++pkts);
          fprintf(stdout,"%lu %hd %hd %hd %hd %hd %hd %hd %hd %hd %d",
            M2->IMU_DATA.Time,
            M2->IMU_DATA.pqr[0],M2->IMU_DATA.pqr[1],M2->IMU_DATA.pqr[2],
            M2->IMU_DATA.axyz[0],M2->IMU_DATA.axyz[1],M2->IMU_DATA.axyz[2],
            M2->IMU_DATA.mxyz[0],M2->IMU_DATA.mxyz[1],M2->IMU_DATA.mxyz[2],
            M2->IMU_DATA.Flags);
          fprintf(stdout," %d %lu %hu %hu %ld %ld %ld %ld %ld %ld %hu %hu %hu\n",
            M2->GPS_PV.updated,
            M2->GPS_PV.Time,M2->GPS_PV.Week,M2->GPS_PV.Details,
            M2->GPS_PV.Pos[0],M2->GPS_PV.Pos[1],M2->GPS_PV.Pos[2],
            M2->GPS_PV.Vel[0],M2->GPS_PV.Vel[1],M2->GPS_PV.Vel[2],
            M2->GPS_PV.PDOP,M2->GPS_PV.PAcc,M2->GPS_PV.SAcc);
          M2->GPS_PV.updated=0;
          break;
          
          #ifdef NMEA
          //If NMEA is defined, generate a NMEA output message for every NAV_PV
          //message received.  The routines in NMEAOut.c will generate the messages
          //and store them into the NMEA queue.  Then, we must stream the data from
          //the queue into the file.
          case 12:  //NAV_PV
            {
            NMEAOutput(hwNMEA, M2);
            while((i=mQueueReadChar(hrNMEA)) >= 0 )
              fputc(i, fpnmea);
            break;
            }
          #endif
          }
        }
      }
    }

  return 0;
  }
//---------------------------------------------------------------------------
