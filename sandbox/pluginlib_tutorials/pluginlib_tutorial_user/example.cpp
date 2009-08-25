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

#include "ros/console.h"

#include "pluginlib_tutorial_interfaces/polygon.h"
#include "pluginlib_tutorial_interfaces/shape.h"
#include "pluginlib/class_loader.h"

int main() {
  
  pluginlib::ClassLoader<polygon> cl("pluginlib_tutorial_interfaces", "polygon");

  ROS_INFO("Created Class Loader of polygon");
  ROS_INFO("Available plugins are:");
  std::vector<std::string> plugins = cl.getDeclaredClasses();
  for (std::vector<std::string>::iterator it = plugins.begin(); it != plugins.end() ; ++it)
  {
    ROS_INFO("%s is in package %s and is of type %s", it->c_str(), cl.getClassPackage(*it).c_str(), cl.getClassType(*it).c_str());
    ROS_INFO("It does \"%s\"", cl.getClassDescription(*it).c_str());
    //ROS_INFO("It is found in library %s", cl.getClassLibraryPath(*it).c_str());
  }


  if (cl.loadLibraryForClass("square"))
  {
    ROS_INFO("Loaded library with plugin square inside");
  }
  else
  {
    ROS_INFO("Failed to load library with plugin square inside");
  }
  
  if (cl.isClassLoaded("square"))
  { 
    ROS_INFO("Can create square");
    polygon * poly = cl.createClassInstance("square");
    // use the class
    poly->set_side_length(7);
    ROS_INFO("The square area is: %.2f", poly->area());
  }
  else ROS_INFO("Square Class not loaded");
  
  ROS_INFO("Created square, trying triangle next");
  if (cl.isClassLoaded("triangle"))
  { 
    polygon * tri = cl.createClassInstance("triangle");

  // use the class
  tri->set_side_length(7);
  ROS_INFO("The triangle area is: %.2f", tri->area());
  }
  else ROS_INFO("Triangle Class not loaded");
  
  
  if (!cl.loadLibraryForClass("line"))
    ROS_ERROR("Correctly failed to load line in polygon loader");

  pluginlib::ClassLoader<shape> ph("pluginlib_tutorial_interfaces", "shape");

  if (!ph.loadLibraryForClass("line"))
    ROS_ERROR("Failed to load line");

  if (ph.isClassLoaded("line"))
    {
      shape * sh = ph.createClassInstance("line");
      // use the class
      sh->set_side_length(7);
      ROS_INFO("The line area is: %.2f", sh->area()); 
    }
    else
      ROS_ERROR("Cloudn't find line lib");

  //std::vector<std::string> libs = cl.getLoadedLibraries();
  //ROS_INFO("Libraries in cl are:");
  //for (std::vector<std::string>::iterator it = libs.begin(); it != libs.end() ; ++it)
  //{
  //  ROS_INFO("Library name %s", it->c_str());
  //  std::vector<std::string> plugins = cl.getClassesInLibrary(*it);
  //  ROS_INFO("  With plugins:");
  //  for (std::vector<std::string>::iterator it2 = plugins.begin(); it2 != plugins.end() ; ++it2)
  //  {
  //    ROS_INFO("   %s", it2->c_str());
  //  }
  //}

  return 0;
}
