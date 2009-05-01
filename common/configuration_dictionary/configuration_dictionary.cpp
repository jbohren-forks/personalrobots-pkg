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

bool ConfigurationDictionary::hasKey(const std::string &key){
  if(bool_values.count(key) > 0 || int_values.count(key) > 0 || double_values.count(key) > 0 || string_values.count(key) > 0)
    return true;
  return false;
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

void MutableConfigurationDictionary::deleteParam(const std::string &key){
  bool_values.erase(key);
  int_values.erase(key);
  double_values.erase(key);
  string_values.erase(key);
}
