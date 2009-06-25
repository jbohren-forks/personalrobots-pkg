#ifndef POLYGON_HPP
#define POLYGON_HPP

template<class T>
class base_polygon
{
public:
  base_polygon(): side_length_(0) {};
protected:
  T side_length_;
};

class polygon : public base_polygon<double>{

public:
  polygon(){}

  virtual ~polygon() {}

  void set_side_length(double side_length) {
    side_length_ = side_length;
  }

  virtual double area() const = 0;
};

// the types of the class factories
typedef polygon* create_t();
typedef void destroy_t(polygon*);

#endif
