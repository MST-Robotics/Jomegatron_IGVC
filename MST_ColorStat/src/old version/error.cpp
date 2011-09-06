// Programmer: ANDREW REIDMEYER    Date: 5-12-11
// File:
// Descr:

#include "error.h"

#include <iostream>
using namespace std;

void error::update()
{
  refRGB[0] = in->rgb[ IMAGE_HEIGHT/2 ][ IMAGE_WIDTH/2 ][0];
  refRGB[1] = in->rgb[ IMAGE_HEIGHT/2 ][ IMAGE_WIDTH/2 ][1];
  refRGB[2] = in->rgb[ IMAGE_HEIGHT/2 ][ IMAGE_WIDTH/2 ][2];
  
  for(int j = 0; j < IMAGE_HEIGHT; j++)
  {
    for(int i = 0; i < IMAGE_WIDTH; i++)
    {
      int r = in->rgb[j][i][0];
      int g = in->rgb[j][i][1];
      int b = in->rgb[j][i][2];
      int level = (IMAGE_WIDTH - 1)*(r + g + b)/3.0/255.0;
      int L = 2*(r + g + b);
      float R,G,B;
      if(L)
      {
        R = (float)(2*r - g - b)/L;
        G = (float)(2*g - r - b)/L;
        B = (float)(2*b - r - g)/L;
      }
      else { R = 0; G = 0; B = 0; }
      R += 1.0; G += 1.0; B += 1.0;
      R *= (IMAGE_WIDTH/2 - 1);
      G *= (IMAGE_WIDTH/2 - 1);
      B *= (IMAGE_WIDTH/2 - 1);
      double L1 = (double)stats->lev[level]/stats->max;
      double R1 = (double)stats->red[(int)R]/stats->rmax;
      double G1 = (double)stats->green[(int)G]/stats->gmax;
      double B1 = (double)stats->blue[(int)B]/stats->bmax;
      const float mult = 32;
      L1 *= mult; if(L1 > 1) L1 = 1.0;
      R1 *= mult; if(R1 > 1) R1 = 1.0;
      G1 *= mult; if(G1 > 1) G1 = 1.0;
      B1 *= mult; if(B1 > 1) B1 = 1.0;
      double neterror = sqrt( sqrt( L1*R1*G1*B1 ) );
      
      //neterror += levelerror( in->rgb[j][i] );
      neterror = 1 - neterror;
      neterror*=255;
      out.rgb[j][i][0] = (unsigned char)neterror;
      out.rgb[j][i][1] = (unsigned char)neterror;
      out.rgb[j][i][2] = (unsigned char)neterror;
    }
  } 
  
  //noise reduction
  const int r = 2;
  out2 = out;
  for(int j = 0; j < IMAGE_HEIGHT; j++)
  {
    for(int i = 0; i < IMAGE_WIDTH; i++)
    {
      if(out.rgb[j][i][1])
      {
        int sum = 0;
        for(int y = j - r; y <= j + r; y++)
        {
          for(int x = i - r; x <= i + r; x++)
          {
            sum += out.rgb[y][x][1];
          }
        }
        int val = 0;
        {
          int size = 2*r + 1; size*=size;
          float t = ((float)sum/size/255);
          val = out.rgb[j][i][1]*t*t;
          out2.rgb[j][i][1] = val;   
          out2.rgb[j][i][0] = val; 
          out2.rgb[j][i][2] = val;          
        }
      }
    }
  }  
  
  return;
}

short error::levelerror(unsigned char* in)
{
  short level = sqrtf( in[0]*in[0] + in[1]*in[1] + in[2]*in[2] );
  short levRef = sqrtf( refRGB[0]*refRGB[0] + refRGB[1]*refRGB[1] + refRGB[2]*refRGB[2] );
  return (255*exp( -(float)(level - levRef)*(level - levRef) / levelk / levelk ) );
}
