#ifndef MQUNITS_H
#define MQUNITS_H

#include <cmath>

namespace mqmath
{

class units
{
public:
  static inline const double INCHES()  { return 0.0254; }
  static inline const double METERS()  { return 1;      }
  static inline const double DEGREES() { return M_PI / 180; }
};



}

#endif

