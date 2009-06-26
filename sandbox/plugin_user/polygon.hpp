#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <string>
#include <set>
#include <vector>
#include <iostream>

#include "boost/algorithm/string.hpp"

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


std::set<std::string> findPlugins(const std::string & package, const std::string& type)
{
  std::set<std::string> output;
    
  std::string cmd = "rospack depends-on1 plugin_user | xargs -L1 rospack export --lang=filter --attrib=" + type;
  std::cout << cmd << std::endl;
  FILE* pipe = popen(cmd.c_str(), "r");
  std::string output_str;
    
  if (pipe)
  {
    char rospack_output[1024];
    while (fgets(rospack_output, 1024, pipe))
    {
      output_str += rospack_output;
    }
      
    pclose(pipe);
  }

  size_t trailing_iterator;
  size_t found_iterator;

  std::cout << output_str << std::endl;
  
  std::vector<std::string> output_vec;
  
  boost::split(output_vec, output_str, boost::is_any_of("\n "));
  
  for (std::vector<std::string>::iterator it = output_vec.begin(); it != output_vec.end() ; it++)
  {
    if (it->size() == 0) continue;
    output.insert(*it);
    std::cout << "adding " << *it << std::endl;
  };
  return output;

};



#endif
