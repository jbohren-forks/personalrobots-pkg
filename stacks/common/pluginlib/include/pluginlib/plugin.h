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
#ifndef PLUGINLIB_PLUGIN_H_
#define PLUGINLIB_PLUGIN_H_
namespace pluginlib {
  /**
   * @class Plugin
   * @brief Storage for information about a given plugin
   */
  class Plugin
  {
    public:
      /**
       * @brief  Constructor for a Plugin
       * @param name The name of the plugin 
       * @param class_name The name of the derived class of the plugin
       * @param type The type of the plugin, corresponds to the type of the base class
       * @param package The package the plugin lives in
       * @param description A description for the plugin
       * @param library_path The path to the containing library for the plugin
       */
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
};
#endif
