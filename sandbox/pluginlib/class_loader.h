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
#ifndef PLUGINLIB_PLUGIN_LOADER_H
#define PLUGINLIB_PLUGIN_LOADER_H

#include <iostream>

#include "Poco/ClassLibrary.h"
#include "Poco/ClassLoader.h"
#include "ros/package.h"
#include "tinyxml/tinyxml.h"
#include <vector>
#include <map>

#include "boost/filesystem.hpp"

namespace fs = boost::filesystem;

#define BEGIN_PLUGIN_LIST(base_class) \
  POCO_BEGIN_MANIFEST(base_class)

#define REGISTER_PLUGIN(plugin_class) \
  POCO_EXPORT_CLASS(plugin_class) 

#define END_PLUGIN_LIST \
  POCO_END_MANIFEST


namespace pluginlib
{

  template <class T>
    class PluginLoader 
    {
      private:
        typedef std::map<std::string, unsigned int> LibraryCountMap;


        class Plugin
        {
          public:
            Plugin(const std::string& name, const std::string& class_name, const std::string& type, const std::string& package, 
                const std::string& description, const std::string& library_path):
              name_(name), 
              class_name_(class_name),
              type_(type),
              package_(package),
              description_(description), 
              library_path_ (library_path){};
            std::string name_;
            std::string class_name_;
            std::string type_;
            std::string package_;
            std::string description_;
            std::string library_path_;

        };

        typedef typename std::map<std::string, Plugin>::iterator PluginMapIterator;

      public:

        PluginLoader(std::string package, std::string plugin_type)
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
                std::string plugin_class_name = plugin->Attribute("class");

                plugins_available_.insert(std::pair<std::string, Plugin>(plugin_name, Plugin(plugin_name, plugin_class_name, plugin_type, package_name, description_str, full_library_path.string())));


                //step to next plugin
                plugin = plugin->NextSiblingElement( "plugin" );
              }


            }
          }

        }

        bool loadPlugin(const std::string & plugin_name)
        {
          std::string library_path;
          PluginMapIterator it = plugins_available_.find(plugin_name);
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
            loadPluginLibrary(library_path);
            //poco_assert (poco_class_loader_.isLibraryLoaded(library_path));
            //zpoco_check_ptr (poco_class_loader_.findManifest(library_path)); 
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

        ~PluginLoader()
        {
          for (LibraryCountMap::iterator it = loaded_libraries_.begin(); it != loaded_libraries_.end(); ++it)
          {
            if ( it->second > 0)
              unloadPluginLibrary(it->first);
          }
        };


        bool isPluginLoaded(const std::string& name)
        {
          try
          {
            return poco_class_loader_.canCreate(getPluginClass(name));
          }
          catch (Poco::RuntimeException &ex)
          {
            return false;
          }
        };

        std::vector<std::string> getDeclaredPlugins()
        {
          std::vector<std::string> plugin_names;
          for (PluginMapIterator it = plugins_available_.begin(); it != plugins_available_.end(); ++it)
          {
            plugin_names.push_back(it->first);
          }    
          return plugin_names;
        };

        std::string getPluginClass(const std::string& plugin_name){
          PluginMapIterator it = plugins_available_.find(plugin_name);
          if (it != plugins_available_.end())
            return it->second.class_name_;
          return "";
        }

        std::string getPluginDescription(const std::string& plugin_name)
        {
          PluginMapIterator it = plugins_available_.find(plugin_name);
          if (it != plugins_available_.end())
            return it->second.description_;
          return "";
        };

        std::string getPluginType(const std::string& plugin_name)
        {
          PluginMapIterator it = plugins_available_.find(plugin_name);
          if (it != plugins_available_.end())
            return it->second.type_;
          return "";
        };

        std::string getPluginLibraryPath(const std::string& plugin_name)
        {
          PluginMapIterator it = plugins_available_.find(plugin_name);
          if (it != plugins_available_.end())
            return it->second.library_path_;
          return "";
        };

        std::string getPluginPackage(const std::string& plugin_name)
        {
          PluginMapIterator it = plugins_available_.find(plugin_name);
          if (it != plugins_available_.end())
            return it->second.package_;
          return "";
        };

        T* createPluginInstance(const std::string& name, bool auto_load_plugin = true)
        {
          if ( auto_load_plugin && !isPluginLoaded(name))
            if(!loadPlugin(name))
            {
              //\todo THROW HERE
              std::cerr <<"Failed to auto load library" << std::endl;
              return NULL;
            }

          //\todo rethrow with non poco Exceptions
          return poco_class_loader_.create(getPluginClass(name));
        };

        void unloadPluginLibrary(const std::string& library_path)
        {
          LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
          if (it == loaded_libraries_.end())
          {
            std::cerr << "unable to unload library which is not loaded" << std::endl;
            return;
          }
          else if (it-> second > 1)
            (it->second)--;
          else 
            poco_class_loader_.unloadLibrary(library_path);

        }

        void loadPluginLibrary(const std::string& library_path)
        {
          poco_class_loader_.loadLibrary(library_path);
          LibraryCountMap::iterator it = loaded_libraries_.find(library_path);
          if (it == loaded_libraries_.end())
            loaded_libraries_[library_path] = 1;  //for correct destruction and access
          else 
            loaded_libraries_[library_path] = loaded_libraries_[library_path] + 1;
        }

        std::vector<std::string> getPluginsInLibrary(const std::string & library_path)
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
        };

        std::vector<std::string> getLoadedLibraries()
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
        };

      private:

        //used for proper unloading of automatically loaded libraries
        LibraryCountMap loaded_libraries_;


        // map from library to plugin's descriptions  
        // This is all available plugins found in xml
        std::map<std::string, Plugin> plugins_available_;

        Poco::ClassLoader<T> poco_class_loader_;  
    };
};
#endif //PLUGINLIB_PLUGIN_LOADER_H
