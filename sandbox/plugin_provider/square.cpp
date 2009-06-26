#include "polygon.hpp"
//#include "Poco/SharedLibrary.h" 
#include "Poco/ClassLibrary.h"

#include <cmath>

class square : public polygon {
public:
  virtual double area() const {
    return side_length_ * side_length_;
  }
};


POCO_BEGIN_MANIFEST(polygon)

POCO_EXPORT_CLASS(square)

POCO_END_MANIFEST 
