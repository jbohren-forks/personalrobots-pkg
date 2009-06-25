#include "polygon.hpp"
#include <cmath>

class square : public polygon {
public:
  virtual double area() const {
    return side_length_ * side_length_;
  }
};


// the class factories

extern "C" polygon* create() {
  return new square;
}

extern "C" void destroy(polygon* p) {
  delete p;
}
