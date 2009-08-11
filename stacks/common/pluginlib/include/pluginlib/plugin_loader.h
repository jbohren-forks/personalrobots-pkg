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

#include "pluginlib/plugin.h"

#include "Poco/ClassLoader.h"
#include "ros/package.h"
#include "tinyxml/tinyxml.h"
#include <vector>
#include <map>

#include "boost/filesystem.hpp"

namespace fs = boost::filesystem;



namespace pluginlib
{

  /**
   * @class PluginLoader
   * @brief A class to help manage and load plugins
   */
  template <class T>
    class PluginLoader 
    {
      private:
        typedef std::map<std::string, unsigned int> LibraryCountMap;

      public:
        typedef typename std::map<std::string, Plugin>::iterator PluginMapIterator;

      public:
        /**
         * @brief  Constructor for a PluginLoader
         * @param package The package containing the base class for the plugins 
         * @param plugin_type The type or base class of the plugins to be loaded
         */
        PluginLoader(std::string package, std::string plugin_type);

        /**
         * @brief  Attempts to load a plugin with a given name
         * @param plugin_name The name of the plugin to load
         * @return True if the plugin and its associated library were successfully loaded, false otherwise
         */
        bool loadPlugin(const std::string & plugin_name);

        /**
         * @brief  Destructor for PluginLoader 
         */
        ~PluginLoader();


        /**
         * @brief Checks if a given plugin is currently loaded
         * @param  name The name of the plugin to query
         * @return True if the plugin is loaded, false otherwise
         */
        bool isPluginLoaded(const std::string& name);

        /**
         * @brief  Returns a list of all available plugins for this PluginLoader's plugin type
         * @return A vector of strings corresponding to the names of all available plugins
         */
        std::vector<std::string> getDeclaredPlugins();

        /**
         * @brief  Given the name of a plugin, returns the name of the derived class associated with it
         * @param plugin_name The name of the plugin 
         * @return The name of the associated derived class
         */
        std::string getPluginClass(const std::string& plugin_name);

        /**
         * @brief  Given the name of a plugin, returns its description
         * @param plugin_name The name of the plugin 
         * @return The description of the plugin
         */
        std::string getPluginDescription(const std::string& plugin_name);

        /**
         * @brief  Given the name of a plugin, returns the name of the base class or type associated with it
         * @param plugin_name The name of the plugin 
         * @return The name of the associated base class or type
         */
        std::string getPluginType(const std::string& plugin_name);

        /**
         * @brief  Given the name of a plugin, returns the path to its associated library
         * @param plugin_name The name of the plugin 
         * @return The path to the associated library
         */
        std::string getPluginLibraryPath(const std::string& plugin_name);

        /**
         * @brief  Given the name of a plugin, returns name of the containing package
         * @param plugin_name The name of the plugin 
         * @return The name of the containing package
         */
        std::string getPluginPackage(const std::string& plugin_name);

        /**
         * @brief  Creates an instance of a desired plugin, optionally loading the associated library automatically if necessary
         * @param  name The name of the plugin to load
         * @param  auto_load_plugin Specifies whether or not to automatically load the library containing the plugin, set to true by default
         * @exception std::runtime_error Thrown when the library cannot be loaded or the plugin cannot be instantiated 
         * @return An instance of the plugin
         */
        T* createPluginInstance(const std::string& name, bool auto_load_plugin = true);

        /**
         * @brief  Unloads a previously dynamically loaded lobrary
         * @param library_path The library to unload
         * @return True if the library was successfully unloaded, false otherwise
         */
        bool unloadPluginLibrary(const std::string& library_path);

        /**
         * @brief  Dynamicaly loads a library
         * @param library_path The library to unload
         * @return True if the library was successfully loaded, false otherwise
         */
        bool loadPluginLibrary(const std::string& library_path);

        /**
         * @brief  Returns the names of the plugins that are available in a given library
         * @param  library_path The path to the library
         * @return A vector of strings corresponding to the names of the plugins in the library
         */
        std::vector<std::string> getPluginsInLibrary(const std::string & library_path);

        /**
         * @brief  Returns the libraries that are currently loaded
         * @return A vector of strings corresponding to the names of loaded libraries
         */
        std::vector<std::string> getLoadedLibraries();

      private:
        /**
         * @brief  Helper function for loading a shared library
         * @param  library_path the path to the library to load
         */
        void loadPluginLibraryInternal(const std::string& library_path);

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
