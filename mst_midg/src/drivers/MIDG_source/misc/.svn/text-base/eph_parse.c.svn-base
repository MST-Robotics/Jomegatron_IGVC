//Example for parsing a MIDG II GPS_EPH message.

void ParseEphemeris(char *eph_message)
  {
  //eph_message is the contents of the lbuf field of the M2 message
  //parser.  It should be the ephemeris reply starting with the
  //ID byte (Id, count, payload0, payload1, ..., payloadN).
  //The payload of an ephemeris reply is as follows:
  //    byte 0: SVID (1-32)
  //  If no bytes follow the SVID, then no valid ephemeris is available
  //  for the SV.  If valid ephemeris is available, then the message
  //  continues with...
  //    bytes 1-4: HOW (handover word of 1st subframe, big endian).
  //               If zero, invalid ephemeris follows.
  //    bytes 5-77:
  //            Subframes 1-3 of GPS Nav data, each frame is 8 words,
  //            each word is 24 bits (3 bytes), big-endian.
  //            3 frames * 8 words/frame * 3 bytes/word = 72 bytes.

  typedef struct
    {
    short ttrefresh;      //time till refresh (every 15 minutes?) (-1 if no eph)
    unsigned char IODE;
    unsigned char SV;     //SV this corresponds to
    float Crs;
    float dn;
    float M0;
    float Cuc;
    float ec;
    float Cus;
    float rootA;
    float Toe;
    float Cic;
    float W0;
    float Cis;
    float i0;
    float Crc;
    float w;
    float Wdot;
    float Idot;
    } tGPSEph;


  //The payload pointer points to the start of the eph message payload
  unsigned char *sf2 = eph_message+4+5+24;
  unsigned char *sf3 = eph_message+4+5+48;
  unsigned long ud;
  long int  ld;
  short int h;
  unsigned short uh;
  tGPSEph eph;

  //assign the SV ID
  eph.SV = eph_message[4];

  //If the handover word of subframe 1 is zero, then invalid ephemeris
  //data follows.
  ld = (eph_message[5]<<24) | (eph_message[6]<<16) | (eph_message[7]<<8) | eph_message[8];
  if(!ld)
    return;

  //Also, check to make sure that the IODE from sub-frames 2 and 3 match.
  //If not, we cannot use this.
  if(sf2[0] != sf3[21])
    return;

  //Now, start extraction of the ephemeris data.
  //Subframe 2
  eph.IODE = sf2[0];

  h = ((sf2[1]<<8) | sf2[2]);
  eph.Crs = h*0.03125;           //2^-5

  h = ((sf2[3]<<8) | sf2[4]);
  eph.dn = h*3.57157734197e-13;        //2^-43 * pi

  ld = ((sf2[5]<<24) | (sf2[6]<<16) |(sf2[7]<<8) | sf2[8]);
  eph.M0 = ld*1.46291807927e-9;        //2^-31 * pi

  h = ((sf2[9]<<8) | sf2[10]);
  eph.Cuc = h*1.862645e-9;       //2^-29

  ud = ((sf2[11]<<24) | (sf2[12]<<16) |(sf2[13]<<8) | sf2[14]);
  eph.ec = ud*1.1641532e-10;     //2^-33

  h = ((sf2[15]<<8) | sf2[16]);
  eph.Cus = h*1.862645e-9;       //2^-29

  ud = ((sf2[17]<<24) | (sf2[18]<<16) |(sf2[19]<<8) | sf2[20]);
  eph.rootA = ud*1.9073486e-6;   //2^-19

  uh = ((sf2[21]<<8) | sf2[22]);
  eph.Toe = uh*16.0;              //2^4

  //Now for subframe 3 data
  h = ((sf3[0]<<8) | sf3[1]);
  eph.Cic = h*1.862645e-9;       //2^-29

  ld = ((sf3[2]<<24) | (sf3[3]<<16) |(sf3[4]<<8) | sf3[5]);
  eph.W0 = ld*1.46291807927e-9;        //2^-31 * pi

  h = ((sf3[6]<<8) | sf3[7]);
  eph.Cis = h*1.862645e-9;       //2^-29

  ld = ((sf3[8]<<24) | (sf3[9]<<16) |(sf3[10]<<8) | sf3[11]);
  eph.i0 = ld*1.46291807927e-9;        //2^-31 * pi

  h = ((sf3[12]<<8) | sf3[13]);
  eph.Crc = h*0.03125;           //2^-5

  ld = ((sf3[14]<<24) | (sf3[15]<<16) |(sf3[16]<<8) | sf3[17]);
  eph.w = ld*1.46291807927e-9;         //2^-31 * pi

  //Wdot is 24 bits.  We will construct it in the upper bytes to allow
  //it to be signed and then multiply by 2^-51 instead of 2^-43.
  ld = ((sf3[18]<<24) | (sf3[19]<<16) |(sf3[20]<<8));
  eph.Wdot = ld*1.3951473992e-15;      //2^-51 * pi

  //idot is a 14 bit signed number.  We'll leave it in 16 bits and multiply
  //by 2^-45 instead of 2^43.
  h = ((sf3[22]<<8) | sf3[23]);
  eph.Idot = h*8.9289433549e-14;    //2^-45 * pi
  }
