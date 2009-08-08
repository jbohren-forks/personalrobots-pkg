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

#include "configuration_dictionary.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/param.h>

bool ConfigurationDictionary::getParam(const std::string &key, bool &value){
  if(bool_values.count(key) > 0)
    value = bool_values[key];
    return true;
  return false;
}

bool ConfigurationDictionary::getParam(const std::string &key, int &value){
 if(int_values.count(key) > 0)
    value = int_values[key];
    return true;
  return false;
}

bool ConfigurationDictionary::getParam(const std::string &key, double &value){
  if(double_values.count(key) > 0)
    value = double_values[key];
    return true;
  return false;
}

bool ConfigurationDictionary::getParam(const std::string &key, std::string &value){
  if(string_values.count(key) > 0)
    value = string_values[key];
    return true;
  return false;
}

bool ConfigurationDictionary::getParam(const std::string &key, ConfigurationDictionary &value){
  if(dictionary_values.count(key) > 0)
    value = *dictionary_values[key];
    return true;
  return false;
}

bool ConfigurationDictionary::hasKey(const std::string &key){
  if(bool_values.count(key) > 0 || int_values.count(key) > 0 || double_values.count(key) > 0 || string_values.count(key) > 0)
    return true;
  return false;
}


std::string ConfigurationDictionary::asYaml(){
  YAML::Emitter emitter;
  emitAsYaml(emitter);
  return emitter.c_str();
}

void ConfigurationDictionary::emitAsYaml(YAML::Emitter &emitter){
  if(bool_values.empty() && int_values.empty() && double_values.empty() && string_values.empty() && dictionary_values.empty())
    return;
  emitter << YAML::BeginMap;

  emit_map(emitter, bool_values);
  emit_map(emitter, int_values);
  emit_map(emitter, double_values);
  emit_map(emitter, string_values);
 
  std::map<std::string, ConfigurationDictionary *>::iterator i = dictionary_values.begin();
  for(; i != dictionary_values.end(); ++i)
  {
    emitter << YAML::Key << i->first;
    emitter << YAML::Value;
    i->second->emitAsYaml(emitter);
  }
 
  emitter << YAML::EndMap;
}


void MutableConfigurationDictionary::setParam(const std::string &key, bool value){
  if(hasKey(key)){
    deleteParam(key);
  }
  bool_values[key] = value;
}

void MutableConfigurationDictionary::setParam(const std::string &key, int value){
  if(hasKey(key)){
    deleteParam(key);
  }
  int_values[key] = value;
}

void MutableConfigurationDictionary::setParam(const std::string &key, double value){
  if(hasKey(key)){
    deleteParam(key);
  }
  double_values[key] = value;
}

void MutableConfigurationDictionary::setParam(const std::string &key, std::string value){
  if(hasKey(key)){
    deleteParam(key);
  }
  string_values[key] = value;
}

void MutableConfigurationDictionary::setParam(const std::string &key, char *value){
  if(hasKey(key)){
    deleteParam(key);
  }
  string_values[key] = std::string(value);
}

void MutableConfigurationDictionary::setParam(const std::string &key, ConfigurationDictionary *value){
  printf("Set dictionary param.\n");
  if(hasKey(key)){
    deleteParam(key);
  }
  dictionary_values[key] = value;
}

void MutableConfigurationDictionary::deleteParam(const std::string &key){
  bool_values.erase(key);
  int_values.erase(key);
  double_values.erase(key);
  string_values.erase(key);
  dictionary_values.erase(key); //TODO - delete this when it is erased
}

bool MutableConfigurationDictionary::loadFromParamServer(const std::string &key){
  XmlRpc::XmlRpcValue paramValue;
  if(ros::param::get(key, paramValue))//TODO: Check return value for errors
  {
    return loadFromXmlRpcValue(paramValue);
    return true;
  }
  else{
    ROS_DEBUG("Failed to load ConfigurationDictionary from parameter %s", key.c_str());
    return false;
  }
}

bool MutableConfigurationDictionary::loadFromXmlRpcValue(XmlRpcValue &value){
  if(value.getType() != XmlRpc::XmlRpcValue::TypeStruct){
    ROS_WARN("ConfigurationDictionary can only load from a namespace of the param server");
    return 0;
  }
  MutableConfigurationDictionary *d;
  for(XmlRpcValue::ValueStruct::iterator i = value.begin(); i != value.end(); i++){
    switch(i->second.getType()){
    case XmlRpcValue::TypeBoolean:
      setParam(i->first, (bool)i->second);
      break;
    case XmlRpcValue::TypeInt:
      setParam(i->first, (int)i->second);
      break;
    case XmlRpcValue::TypeDouble:
      setParam(i->first, (double)i->second);
      break;
    case XmlRpcValue::TypeString:
      setParam(i->first, (std::string)i->second);
      break;
    case XmlRpcValue::TypeStruct:
      d = new MutableConfigurationDictionary();
      if(d->loadFromXmlRpcValue(i->second)){
        setParam(i->first, d); 
      }
      else{
        delete d;
        return false;
      }
      break;
    default:
      ROS_WARN("Attempted to load ConfigurationDictionary from unsupported type on param server\n");
    }
  }
  return true;
}

bool MutableConfigurationDictionary::loadFromYamlNode(const YAML::Node &node){
  if(node.GetType() != YAML::CT_MAP){
    printf("Error, top level must be a map.");
    return false;
  }
  for(YAML::Iterator it=node.begin();it!=node.end();++it) {
    std::string key, value;
    MutableConfigurationDictionary *d;
    it.first() >> key;
    switch(it.second().GetType()){
    case YAML::CT_SCALAR:
      it.second() >> value;
      setParam(key, value);
      break;
    case YAML::CT_MAP:
      printf("Loading new namespace.\n");
      d = new MutableConfigurationDictionary();
      d->loadFromYamlNode(it.second());
      setParam(key, d);
      break;
    case YAML::CT_NONE:
      printf("Unexpected empty node.\n");
      return false;
    case YAML::CT_SEQUENCE:
      printf("Sequences currently unsupported.\n");
      return false;
    }
  }
  return true;
}

bool MutableConfigurationDictionary::loadFromYaml(std::string &yaml){
  return false;
}

bool MutableConfigurationDictionary::loadFromYamlFile(std::string filename){
  std::ifstream fin("test.yaml"); //TODO: load from proper filename
  //TODO - check that the document loaded properly
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  return loadFromYamlNode(doc);
}
