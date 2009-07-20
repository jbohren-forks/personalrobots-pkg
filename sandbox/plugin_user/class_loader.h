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
#ifndef ROS_CLASSLOADER_H
#define ROS_CLASSLOADER_H

#include <iostream>

#include "Poco/ClassLoader.h"
#include "ros/package.h"

namespace ros
{
template <class T>
class ClassLoader : public Poco::ClassLoader<T>
{
public:
  ClassLoader(std::string package, std::string plugin_type, std::string base_class_name)
  {
    //Pull possible files from manifests of packages which depend on this package and export plugin
    std::set<std::string> paths = findPlugins(package, plugin_type, base_class_name);
    
    //The poco factory for base class T
    for (std::set<std::string>::iterator it = paths.begin(); it != paths.end(); ++it)
    {
      
      
      std::string path = *it;
      
      path.append(Poco::SharedLibrary::suffix());
      //std::cout << "Loading library " << full_path << std::endl;
      try
      {
        this->loadLibrary(path);
        //poco_assert (this->isLibraryLoaded(path));
        //zpoco_check_ptr (this->findManifest(path)); 
        loaded_libraries_.push_back(path);  //for correct destruction and access
      }
      catch (Poco::LibraryLoadException &ex)
      {
        std::cerr << "Failed to load library " << path << ex.what() << std::endl;
      }
      catch (Poco::NotFoundException &ex)
      {
        std::cerr << "Failed to find library " << path << ex.what() << std::endl;
      }
    }
    
  };

  ~ClassLoader()
  {
    for (std::vector<std::string>::iterator it = loaded_libraries_.begin(); it != loaded_libraries_.end(); ++it)
    {
      this->unloadLibrary(*it);
    }
  };


  bool canCreate(const std::string& name)
  {
    try 
    {
      return ((Poco::ClassLoader<T>*)this )->canCreate(name);
    }
    catch (Poco::RuntimeException &ex)
    {
      return false;
    }
  };
  
private:
  std::set<std::string> findPlugins(const std::string & package, const std::string& plugin_style, const std::string& type)
  {
    std::set<std::string> output;
    //\todo make this portable    
    std::string cmd = "rospack depends-on1 plugin_user | xargs -L1 rospack export --lang=" + plugin_style + " --attrib=" + type;
    //std::cout << cmd << std::endl;
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

    //    std::cout << output_str << std::endl;
  
    std::vector<std::string> output_vec;
  
    boost::split(output_vec, output_str, boost::is_any_of("\n "));
  
    for (std::vector<std::string>::iterator it = output_vec.begin(); it != output_vec.end() ; it++)
    {
      if (it->size() == 0) continue;
      output.insert(*it);
      //std::cout << "adding " << *it << std::endl;
    };
    return output;

  };

  //used for proper unloading of automatically loaded libraries
  std::vector<std::string> loaded_libraries_;
  
};

}
#endif //ROS_CLASSLOADER_H
