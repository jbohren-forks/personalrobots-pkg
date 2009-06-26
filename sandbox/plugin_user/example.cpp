/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
  cout << "The square area is: " << poly->area() << '\n';

  polygon * tri = cl.create("triangle");

  // use the class
  tri->set_side_length(7);
  cout << "The triangle area is: " << tri->area() << '\n';

  

}
