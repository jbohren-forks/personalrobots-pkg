#include "polygon.hpp"
#include <iostream>

#include "Poco/ClassLoader.h"



int main() {
  using std::cout;
  using std::cerr;

  std::string path("./lib/libtriangle");
  
  path.append(Poco::SharedLibrary::suffix());
  std::cout << path << std::endl;

  Poco::ClassLoader<polygon> cl;
  
  cl.loadLibrary(path);
  
  poco_assert (cl.isLibraryLoaded(path));
  
  poco_check_ptr (cl.findManifest(path)); 

  polygon * poly = cl.create("triangle");

  // use the class
  poly->set_side_length(7);
  cout << "The area is: " << poly->area() << '\n';


}
