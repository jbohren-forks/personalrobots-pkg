#include "polygon.hpp"
#include <iostream>

#include "Poco/ClassLoader.h"
#include "ros/package.h"


int main() {
  using std::cout;
  using std::cerr;

  std::set<std::string> paths = findPlugins("plugin_user", "plugin");
  Poco::ClassLoader<polygon> cl;

  for (std::set<std::string>::iterator it = paths.begin(); it != paths.end(); ++it)
  {


    std::string path = *it;
  
  path.append(Poco::SharedLibrary::suffix());
  //  path = ros::package::command();
  //  paths = ros::package::command("export --lang=filter --attrib=plugin plugin_provider");

  std::cout << path << std::endl;

  
  
  cl.loadLibrary(path);
  
  poco_assert (cl.isLibraryLoaded(path));
  
  poco_check_ptr (cl.findManifest(path)); 
  }
  polygon * poly = cl.create("square");

  // use the class
  poly->set_side_length(7);
  cout << "The area is: " << poly->area() << '\n';


}
