/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

//NOTE: this should really never be included on its own, but just in case someone is bad we'll guard  

#ifndef PLUGINLIB_PLUGIN_LOADER_IMP_H_
#define PLUGINLIB_PLUGIN_LOADER_IMP_H_

#include <stdexcept>

namespace pluginlib {
  template <class T>
  PluginLoader<T>::PluginLoader(std::string package, std::string plugin_type)
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
          ROS_ERROR("Failed to find Path Attirbute in library element in %s", it->c_str());
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
            ROS_ERROR("Could not find package name for plugin %s", it->c_str());
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
          std::string plugin_class_name = plugin->Attribute("class");

          plugins_available_.insert(std::pair<std::string, Plugin>(plugin_name, Plugin(plugin_name, plugin_class_name, plugin_type, package_name, description_str, full_library_path.string())));


          //step to next plugin
          plugin = plugin->NextSiblingElement( "plugin" );
        }


      }
    }

  }

  template <class T>
  bool PluginLoader<T>::loadPlugin(const std::string & plugin_name)
  {
    std::string library_path;
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      library_path = it->second.library_path_;
    else
    {
      ROS_ERROR("Couldn't find plugin %s", plugin_name.c_str());
      return false;
    }
    library_path.append(Poco::SharedLibrary::suffix());
    ROS_DEBUG("Loading library %s", library_path.c_str());
    try
    {
      loadPluginLibraryInternal(library_path);
    }
    catch (Poco::LibraryLoadException &ex)
    {
      ROS_ERROR("Failed to load library %s %s", library_path.c_str(), ex.what());
      return false;
    }
    catch (Poco::NotFoundException &ex)
    {
      ROS_ERROR("Failed to find library %s %s", library_path.c_str(), ex.what());
      return false;
    }
    return true;
  }

  template <class T>
  PluginLoader<T>::~PluginLoader()
  {
    for (LibraryCountMap::iterator it = loaded_libraries_.begin(); it != loaded_libraries_.end(); ++it)
    {
      if ( it->second > 0)
        unloadPluginLibrary(it->first);
    }
  }


  template <class T>
  bool PluginLoader<T>::isPluginLoaded(const std::string& name)
  {
    try
    {
      return poco_class_loader_.canCreate(getPluginClass(name));
    }
    catch (Poco::RuntimeException &ex)
    {
      return false;
    }
  }

  template <class T>
  std::vector<std::string> PluginLoader<T>::getDeclaredPlugins()
  {
    std::vector<std::string> plugin_names;
    for (PluginMapIterator it = plugins_available_.begin(); it != plugins_available_.end(); ++it)
    {
      plugin_names.push_back(it->first);
    }    
    return plugin_names;
  }

  template <class T>
  std::string PluginLoader<T>::getPluginClass(const std::string& plugin_name)
  {
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      return it->second.class_name_;
    return "";
  }

  template <class T>
  std::string PluginLoader<T>::getPluginDescription(const std::string& plugin_name)
  {
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      return it->second.description_;
    return "";
  }

  template <class T>
  std::string PluginLoader<T>::getPluginType(const std::string& plugin_name)
  {
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      return it->second.type_;
    return "";
  }

  template <class T>
  std::string PluginLoader<T>::getPluginLibraryPath(const std::string& plugin_name)
  {
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      return it->second.library_path_;
    return "";
  }

  template <class T>
  std::string PluginLoader<T>::getPluginPackage(const std::string& plugin_name)
  {
    PluginMapIterator it = plugins_available_.find(plugin_name);
    if (it != plugins_available_.end())
      return it->second.package_;
    return "";
  }

  template <class T>
  T* PluginLoader<T>::createPluginInstance(const std::string& name, bool auto_load_plugin)
  {
    if ( auto_load_plugin && !isPluginLoaded(name))
      if(!loadPlugin(name))
      {
        //\todo THROW HERE
        ROS_ERROR("Failed to auto load library");
        return NULL;
        throw std::runtime_error("Failed to auto load library for plugin " + name + ".");
      }

    try{
      return poco_class_loader_.create(getPluginClass(name));
    }
    catch(const Poco::RuntimeException& ex){
      throw std::runtime_error(ex.what());
    }
  } 

  template <class T>
  bool PluginLoader<T>::unloadPluginLibrary(const std::string& library_path)
  {
    LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
    if (it == loaded_libraries_.end())
    {
      ROS_DEBUG("unable to unload library which is not loaded");
      return false;
    }
    else if (it-> second > 1)
      (it->second)--;
    else 
      poco_class_loader_.unloadLibrary(library_path);

    return true;

  }

  template <class T>
  bool PluginLoader<T>::loadPluginLibrary(const std::string& library_path){
    try
    {
      loadPluginLibraryInternal(library_path);
    }
    catch (Poco::LibraryLoadException &ex)
    {
      return false;
    }
    catch (Poco::NotFoundException &ex)
    {
      return false;
    }
    return true;
  }

  template <class T>
  void PluginLoader<T>::loadPluginLibraryInternal(const std::string& library_path) {
    poco_class_loader_.loadLibrary(library_path);
    LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
    if (it == loaded_libraries_.end())
      loaded_libraries_[library_path] = 1;  //for correct destruction and access
    else 
      loaded_libraries_[library_path] = loaded_libraries_[library_path] + 1;
  }

  template <class T>
  std::vector<std::string> PluginLoader<T>::getPluginsInLibrary(const std::string & library_path)
  {
    std::vector<std::string> plugin_names;


    const Poco::Manifest<T> * manifest = poco_class_loader_.findManifest(library_path);
    if (manifest == NULL)
      return plugin_names;

    for (typename Poco::Manifest<T>::Iterator it = manifest->begin(); it != manifest->end(); ++it)
    {
      plugin_names.push_back(it->name());
    }
    return plugin_names;
  }

  template <class T>
  std::vector<std::string> PluginLoader<T>::getLoadedLibraries()
  {
    std::vector<std::string> library_names;

    /*
       \todo find a way to get ths out of poco 
       for (typename Poco::ClassLoader<T>::Iterator it = poco_class_loader_.begin(); it != poco_class_loader_.end(); ++it)
       {
       library_names.push_back(it->second->className());
       }
       return library_names;
       */
    LibraryCountMap::iterator it;
    for (it = loaded_libraries_.begin(); it != loaded_libraries_.end(); it++)
    { 
      if (it->second > 0)
        library_names.push_back(it->first);
    }
    return library_names;
  }
};
#endif
