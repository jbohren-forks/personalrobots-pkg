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
#include "tinyxml/tinyxml.h"


#include "boost/filesystem.hpp"

class Plugin
{
public:
  Plugin(const std::string& name, const std::string& type, const std::string& package, 
         const std::string& description, const std::string& library_path):
    name_(name), 
    type_(type),
    package_(package),
    description_(description), 
    library_path_ (library_path){};
  std::string name_;
  std::string type_;
  std::string package_;
  std::string description_;
  std::string library_path_;
  
};

namespace fs = boost::filesystem;

namespace ros
{
template <class T>
class ClassLoader : public Poco::ClassLoader<T>
{
public:
  ClassLoader(std::string package, std::string plugin_type)
  {
    //Pull possible files from manifests of packages which depend on this package and export plugin
    std::vector<std::string> paths;
    
    ros::package::getPlugins(package, plugin_type, paths);
    
    //The poco factory for base class T
    for (std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it)
    {
      TiXmlDocument document;
      document.LoadFile(*it);
      
      TiXmlElement* library = document.FirstChildElement( "library" );
      if ( library )
      {
        std::string library_path = library->Attribute("path");
        if (library_path.size() == 0)
        {
          std::cerr << "Failed to find Path Attirbute in library element in " << it->c_str() << std::endl;
          continue;
        }

        std::string package_name;

        fs::path p(*it);
        fs::path parent = p.parent_path();
        // figure out the package this plugin is part of
        while (true)
        {
          if (fs::exists(parent / "manifest.xml"))
          {
            std::string package = parent.filename();
            std::string package_path = ros::package::getPath(package);
            if (it->find(package_path) == 0)
            {
              package_name = package;
              break;
            }
          }

          parent = parent.parent_path();

          if (parent.string().empty())
          {
            std::cerr << "Could not find package name for plugin" << *it << std::endl;
            break;
          }
        }
        fs::path full_library_path(parent / library_path);

        TiXmlElement* plugin = library->FirstChildElement( "plugin" );
        while (plugin)
        {
          // register plugin here
          TiXmlElement* description = plugin->FirstChildElement( "description" );
          std::string description_str = description->GetText();

          std::string plugin_name = plugin->Attribute("name");
          
          plugins_available_.insert(std::pair<std::string, Plugin>(plugin_name, Plugin(plugin_name, plugin_type, package_name, description_str, full_library_path.string())));
          

          //step to next plugin
          plugin = plugin->NextSiblingElement( "plugin" );
        }
        
        
      }
    }
    
  };
  
  bool loadPlugin(const std::string & plugin_name)
  {
    std::string library_path;
    std::map<std::string, Plugin>::iterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      library_path = it->second.library_path_;
    else
    {
      std::cerr<< "Couldn't find plugin "<<plugin_name << std::endl;
      return false;
    }
    library_path.append(Poco::SharedLibrary::suffix());
    std::cout << "Loading library " << library_path << std::endl;
    try
    {
      this->loadLibrary(library_path);
      //poco_assert (this->isLibraryLoaded(library_path));
      //zpoco_check_ptr (this->findManifest(library_path)); 
      loaded_libraries_.push_back(library_path);  //for correct destruction and access
    }
    catch (Poco::LibraryLoadException &ex)
    {
      std::cerr << "Failed to load library " << library_path << ex.what() << std::endl;
      return false;
    }
    catch (Poco::NotFoundException &ex)
    {
      std::cerr << "Failed to find library " << library_path << ex.what() << std::endl;
      return false;
    }
    return true;
  }
  
  ~ClassLoader()
  {
    for (std::vector<std::string>::iterator it = loaded_libraries_.begin(); it != loaded_libraries_.end(); ++it)
    {
      this->unloadLibrary(*it);
    }
  };


  bool isPluginLoaded(const std::string& name)
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

  //used for proper unloading of automatically loaded libraries
  std::vector<std::string> loaded_libraries_;

  
  // map from library to plugin's descriptions  
  // This is all available plugins found in xml
  std::map<std::string, Plugin> plugins_available_;
           
  
};

}
#endif //ROS_CLASSLOADER_H
