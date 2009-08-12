/*
 * Copyright (C) 2008, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#ifndef CONGIFURATION_DICTONARY_H
#define CONFIGURATION_DICTONARY_H
#include <string>
#include <map>
#include <ros/node.h>
#include <ros/param.h>
#include "yaml-cpp/yaml.h"
#include <stdio.h>

using XmlRpc::XmlRpcValue;

class ConfigurationDictionary{
public:
  bool getParam(const std::string &key, bool &value);
  bool getParam(const std::string &key, int &value);
  bool getParam(const std::string &key, double &value);
  bool getParam(const std::string &key, std::string &value);
  bool getParam(const std::string &key, ConfigurationDictionary &value);

  bool hasKey(const std::string &key);

  std::string asYaml();
  void emitAsYaml(YAML::Emitter &emitter);

protected:
  std::map<std::string, bool> bool_values;
  std::map<std::string, int> int_values;
  std::map<std::string, double> double_values;
  std::map<std::string, std::string> string_values;
  std::map<std::string, ConfigurationDictionary *> dictionary_values;

  template<typename T> void emit_map(YAML::Emitter &emitter, T map){
  typename T::iterator i = map.begin();
  for(; i != map.end(); i++){
    emitter << YAML::Key << i->first;
    emitter << YAML::Value << i->second;
  }
  }
};

class MutableConfigurationDictionary : public ConfigurationDictionary{
public:
  void setParam(const std::string &key, bool value);
  void setParam(const std::string &key, int value);
  void setParam(const std::string &key, double value);
  void setParam(const std::string &key, std::string value);
  void setParam(const std::string &key, char *value);
  void setParam(const std::string &key, ConfigurationDictionary *value);

  void deleteParam(const std::string &key);

  bool loadFromParamServer(const std::string &name);
  bool loadFromXmlRpcValue(XmlRpcValue &value);

  bool loadFromYaml(std::string &yaml);
  bool loadFromYamlFile(std::string filename);
  bool loadFromYamlNode(const YAML::Node &node);
};

#endif