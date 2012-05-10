// Programmer: ANDREW REIDMEYER    Date: 5-12-11
// File:
// Descr:

#ifndef ERROR_H
#define ERROR_H

#include "image.h"
#include <cmath>
#include "stat.h"

class error
{
  public:
  image* in;
  Stat* stats;
  image out;
  image out2;
  
  void update();  
  
  unsigned char refRGB[3];
  int levelk;
  
  error():levelk(64){}
  
  private:
  
  short levelerror(unsigned char* in);
  
  
};


#endif
