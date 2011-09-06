// Programmer: ANDREW REIDMEYER  Date: 5-19-11
// File stat.h
// Descr:

#ifndef STAT_H
#define STAT_H

#include <cmath>
#include "image.h"

#define SIZE 48

class stat
{
  friend class error;
  public:
  stat();
  
  void update();
  
  void setBounds(int xmin,int xmax,int ymin,int ymax);
  
  image* in;
  image out;
  image out2;
  
  bool drawBounds;
  bool reset;
  bool enabled;
  
  int xMin,xMax,yMin,yMax;
  private:
  
  long long lev[IMAGE_WIDTH];

  long long chr[IMAGE_HEIGHT][IMAGE_WIDTH];
  
  long long max;
  long long maxi;
  
  long long chrmax;

};

#endif
