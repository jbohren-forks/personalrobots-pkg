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
#include "shape.hpp"
#include <iostream>

#include "Poco/ClassLoader.h"
#include "ros/package.h"

#include "class_loader.h"

int main() {
  
  ros::ClassLoader<polygon> cl("plugin_user", "polygon");

  std::cout << "Created" << std::endl;
  
  if (cl.loadPlugin("square"))
  {
    std::cout << "asdf Loaded library with plugin square inside" << std::endl;
  }
  else
  {
    std::cout << "asdf Failed to load library with plugin square inside" << std::endl;
  }
  
  if (cl.isPluginLoaded("square"))
  { 
    std::cout << "Can create square";
    polygon * poly = cl.createPluginInstance("square");
    // use the class
    poly->set_side_length(7);
    std::cout << "The square area is: " << poly->area() << '\n';
  }
  else std::cout << "Square Plugin not loaded" << std::endl;
  
  std::cout << "Created square, trying triangle next" << std::endl;
  if (cl.isPluginLoaded("triangle"))
  { 
    polygon * tri = cl.createPluginInstance("triangle");

  // use the class
  tri->set_side_length(7);
  std::cout << "The triangle area is: " << tri->area() << '\n';
  }
  else std::cout << "Triangle Plugin not loaded" << std::endl;
  
  
  if (!cl.loadPlugin("line"))
    std::cerr<< "Correctly failed to load line in polygon loader" << std::endl;

  ros::ClassLoader<shape> ph("plugin_user", "shape");

  if (!ph.loadPlugin("line"))
    std::cerr<<"Failed to load line" << std::endl;

  if ( ph.isPluginLoaded("line"))
    {
      shape * sh = ph.createPluginInstance("line");
      // use the class
      sh->set_side_length(7);
      std::cout << "The line area is: " << sh->area() << '\n';
    }
    else
      std::cerr << "Cloudn't find line lib" << std::endl;

  return 0;
}
