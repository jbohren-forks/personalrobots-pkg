#include "polygon.hpp"
#include <cmath>
#include "Poco/SharedLibrary.h" 
#include "Poco/ClassLibrary.h"

class triangle : public polygon {
public:
  virtual double area() const {
    return side_length_ * side_length_ * sqrt(3) / 2;
  }
};


POCO_BEGIN_MANIFEST(polygon)

POCO_EXPORT_CLASS(triangle)

POCO_END_MANIFEST 
