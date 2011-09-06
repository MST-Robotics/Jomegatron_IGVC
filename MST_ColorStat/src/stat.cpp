// Programmer: ANDREW REIDMEYER  Date: 5-19-11
// File: stat.cpp
// Descr:

#include "stat.h"

#include <iostream>
using namespace std;

stat::stat()
{
  setBounds(IMAGE_WIDTH/2 - SIZE, IMAGE_WIDTH/2 + SIZE, IMAGE_HEIGHT/2 - SIZE, IMAGE_HEIGHT/2 + SIZE);
  drawBounds = true;
  reset = true;
  enabled = true;
}

void stat::update()
{
  if(!enabled) return;
  //clear stats
  if(reset)
  {
    for(int i = 0; i < IMAGE_WIDTH; i++)
    {
      lev[i] = 0;
      for(int j = 0; j < IMAGE_HEIGHT; j++)
      {
        chr[j][i] = 0;        
      }
    }
    max = 0;
    maxi = 0;
    chrmax = 0;
    
    reset = false;
  }
  
  //generate new stats
  for(int j = yMin; j <= yMax; j++)
  {
    for(int i = xMin; i <= xMax; i++)
    {
      int r = in->rgb[j][i][0];
      int g = in->rgb[j][i][1];
      int b = in->rgb[j][i][2];
      //float level = sqrtf(r*r+g*g+b*b);
      float level = IMAGE_WIDTH*(r + g + b)/3.0/255.0;
      {
        int leveli = (int)level;
        int imin,imax; const int w = 5;
        imin = leveli - w; imax = leveli + w;
        if(imin < 0) imin = 0;
        if(imax >= IMAGE_WIDTH) imax = IMAGE_WIDTH - 1;
        for(int k = imin; k <= imax; k++)
        {
          lev[k]++;
          max = ( max > lev[ k ] ? max : lev[ k ] );
          maxi = ( max > lev[ k ] ? maxi : k);
        }
      }
      int L = (r + g + b);
      float A,B;
      if(L)
      {
        B = (float)(2*g - r - b)/(2*L);
        A = (float)(r-b)/L;
      }
      else {A = 0; B = 0;}
      A += 1.0;
      A *= (IMAGE_WIDTH - 1)/2;
      B += 0.5;
      B /= 1.5;
      B *= (IMAGE_HEIGHT - 1);
      chr[ (int)B ][ (int)A ]++;

      chrmax = (chrmax > chr[ (int)B ][ (int)A ] ? chrmax : chr[ (int)B ][ (int)A ] );
    }
  }
  
  //print graph in out
  for(int i = 0; i < IMAGE_SIZE; i++) *( (unsigned char*) out.rgb + i) = 0;
  for(int i = 0; i < IMAGE_SIZE; i++) *( (unsigned char*) out2.rgb + i) = 0;
  
  for(int i = 0; i < IMAGE_WIDTH; i++)
  {
    const float L = maxi;    
    for(int j = 0; j < IMAGE_HEIGHT*lev[i]/max; j++) 
    {
      out.rgb[j][i][1] = 255;  
    } 
  }
  
  //draw chr
  for(int j = 0; j < IMAGE_HEIGHT; j++)
  {
    for(int i = 0; i < IMAGE_WIDTH; i++)
    {
      out2.rgb[j][i][1] = (255*chr[j][i])/chrmax;     
      out2.rgb[j][i][2] = 0;
      out2.rgb[j][i][0] = 0;
    }    
  }
  
}

void stat::setBounds(int xmin,int xmax,int ymin,int ymax)
{
  xMin = xmin;
  xMax = xmax;
  yMin = ymin;
  yMax = ymax;
  if(xMin < 0) xMin = 0;
  if(xMax >= IMAGE_WIDTH) xMax = IMAGE_WIDTH - 1;
  if(yMin < 0) yMin = 0;
  if(yMax >= IMAGE_HEIGHT) yMax = IMAGE_HEIGHT - 1;
  
  return;
}
