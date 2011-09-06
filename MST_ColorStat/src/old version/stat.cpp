// Programmer: ANDREW REIDMEYER  Date: 5-19-11
// File: stat.cpp
// Descr:

#include "stat.h"

#include <iostream>
using namespace std;

stat::stat()
{
  setBounds(IMAGE_WIDTH/2 - SIZE, IMAGE_WIDTH/2 + SIZE, 0, SIZE);
  drawBounds = true;
  reset = true;
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
      red[i] = 0;
      green[i] = 0;
      blue[i] = 0;
    }
    max = 0;
    maxi = 0;
    rmax = 0;
    gmax = 0;
    bmax = 0;
    
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
      float level = (IMAGE_WIDTH - 1)*(r + g + b)/3.0/255.0;
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
      R *= (IMAGE_WIDTH/2-1);
      G *= (IMAGE_WIDTH/2-1);
      B *= (IMAGE_WIDTH/2-1);
      red[ (int)R ]++;
      green[ (int)G ]++;
      blue[ (int)B ]++;
      rmax = ( rmax > red[ (int)R ] ? rmax : red[ (int)R ] );
      gmax = ( gmax > green[ (int)G ] ? gmax : green[ (int)G ] );
      bmax = ( bmax > blue[ (int)B ] ? bmax : blue[ (int)B ] );
    }
  }
  
  //print graph in out
  for(int i = 0; i < IMAGE_SIZE; i++) *( (unsigned char*) out.rgb + i) = 0;
  for(int i = 0; i < IMAGE_SIZE; i++) *( (unsigned char*) out2.rgb + i) = 0;
  
  for(int i = 0; i < IMAGE_WIDTH; i++)
  {
    const float L = maxi;
    float Chr = 2.0*(i - IMAGE_WIDTH/2)/IMAGE_WIDTH;
    
    for(int j = 0; j < IMAGE_HEIGHT*lev[i]/max; j++) 
    {
      out2.rgb[j][i][1] = 255;  
    } 
    for(int j = 2*IMAGE_HEIGHT/3; j < 2*IMAGE_HEIGHT/3+IMAGE_HEIGHT*red[i]/rmax/3; j++)
    {
      int r = L*(2*Chr + 1)/3;
      int g = (L - r)/2;
      int b = g;
      out.rgb[j][i][0] = r;
      out.rgb[j][i][1] = g;
      out.rgb[j][i][2] = b;
    }
    for(int j = IMAGE_HEIGHT/3; j < IMAGE_HEIGHT/3+IMAGE_HEIGHT*green[i]/gmax/3; j++)
    {
      int g = L*(2*Chr + 1)/3;
      int r = (L - g)/2;
      int b = r;
      out.rgb[j][i][0] = r;
      out.rgb[j][i][1] = g;
      out.rgb[j][i][2] = b;
    }
    for(int j = 0; j < IMAGE_HEIGHT*blue[i]/bmax/3; j++)
    {
      int b = L*(2*Chr + 1)/3;
      int g = (L - b)/2;
      int r = g;
      out.rgb[j][i][0] = r;
      out.rgb[j][i][1] = g;
      out.rgb[j][i][2] = b;
    }
    if( i == IMAGE_WIDTH/2)
    for(int j = 0; j < IMAGE_HEIGHT; j++)
      out.rgb[j][i][1] = 128;
    
  }
  
  /*
  if(drawBounds)
  {
    //draw boundaries
    glColor3f(0,1.0,0);
    glBegin(GL_LINE_LOOP);
    glVertex3f( xMin, yMin, 0.5);
    glVertex3f( xMax, yMin, 0.5);
    glVertex3f( xMax, yMax, 0.5);
    glVertex3f( xMin, yMax, 0.5);
    glEnd();
  } */
  
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
