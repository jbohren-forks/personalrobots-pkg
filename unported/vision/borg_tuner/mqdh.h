#ifndef MQDH_H
#define MQDH_H

#include "mqmat.h"
#include <cmath>

namespace mqmath
{

class mqdh : public mqmat<4,4,double>
{
public:
  mqdh(double al, double a, double d, double th)
  {
    *this | cos(th)         | -sin(th)           |  0       |  a
          | sin(th)*cos(al) |  cos(th) * cos(al) | -sin(al) | -sin(al) * d
          | sin(th)*sin(al) |  cos(th) * sin(al) |  cos(al) |  cos(al) * d
          | 0               |  0                 |  0       |  1           ;
  }
};

}

#endif

