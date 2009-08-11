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

#include "ros/console.h"

#include "Poco/ClassLoader.h"
#include "ros/package.h"
#include "tinyxml/tinyxml.h"
#include <vector>
#include <map>

#include "boost/filesystem.hpp"

namespace fs = boost::filesystem;



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

        PluginLoader(std::string package, std::string plugin_type);

        bool loadPlugin(const std::string & plugin_name);

        ~PluginLoader();


        bool isPluginLoaded(const std::string& name);

        std::vector<std::string> getDeclaredPlugins();

        std::string getPluginClass(const std::string& plugin_name);

        std::string getPluginDescription(const std::string& plugin_name);

        std::string getPluginType(const std::string& plugin_name);

        std::string getPluginLibraryPath(const std::string& plugin_name);

        std::string getPluginPackage(const std::string& plugin_name);

        T* createPluginInstance(const std::string& name, bool auto_load_plugin = true);

        void unloadPluginLibrary(const std::string& library_path);

        void loadPluginLibrary(const std::string& library_path);

        std::vector<std::string> getPluginsInLibrary(const std::string & library_path);

        std::vector<std::string> getLoadedLibraries();

      private:

        //used for proper unloading of automatically loaded libraries
        LibraryCountMap loaded_libraries_;


        // map from library to plugin's descriptions  
        // This is all available plugins found in xml
        std::map<std::string, Plugin> plugins_available_;

        Poco::ClassLoader<T> poco_class_loader_;  
    };

};

#include "plugin_loader_imp.h"

#endif //PLUGINLIB_PLUGIN_LOADER_H
