#include <math.h>

#define SemiMaj 6378137.0
#define SemiMin 6356752.3142451793
/*first and second numerical eccentricity*/
#define e1sqr ((SemiMaj*SemiMaj - SemiMin*SemiMin) / (SemiMaj*SemiMaj))
#define e2sqr ((SemiMaj*SemiMaj - SemiMin*SemiMin) / (SemiMin*SemiMin))

//=====================================================================
//                          LLA2ECEF
//=====================================================================

void LLA2ECEF(double lat, double lon, double alt,
              double *x,  double *y,  double *z)
  {
  //lat,lon in radians, x,y,z,alt in meters
  double N;
  N = SemiMaj / sqrt(1.0 - e1sqr * sin(lat) * sin(lat));

  *x = (N + alt) * cos(lat) * cos(lon);
  *y = (N + alt) * cos(lat) * sin(lon);
  *z = (N * (1.0 - e1sqr) + alt) * sin(lat);
  }


//=====================================================================
//                          ECEF2LLA
//=====================================================================

void ECEF2LLA(double x,    double y,    double z,
              double *lat, double *lon, double *alt)
  {
  //lat,lon in radians, x,y,z,alt in meters
  long double p, T, sT, cT, N;
  p = sqrt(x*x + y*y);
  T = atan2(z * SemiMaj ,(p * SemiMin));
  sT = sin(T);
  cT = cos(T);
  *lat = atan2((z + e2sqr * SemiMin * sT * sT * sT),(p - e1sqr * SemiMaj * cT * cT * cT));
  if (x == 0.0)
    *lon = M_PI / 2.0;
  else
    *lon = atan2(y,x);
  N = SemiMaj / sqrt(1.0 - e1sqr * sin(*lat) * sin(*lat));
  *alt = (p / cos(*lat)) - N;
  }

