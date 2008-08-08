/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
#ifndef ROS_MISC_FACTORY_H
#define ROS_MISC_FACTORY_H

#include <string>
#include <map>
#include <vector>

template <class BaseResult,
          class Constructor = BaseResult* (*)()>
class Factory
{
public:
  static Factory<BaseResult>& instance()
  {
    static Factory<BaseResult> *instance = NULL;
    if (instance == NULL)
      instance = new Factory<BaseResult>;
    return *instance;
  }

  BaseResult *create(const std::string &name)
  {
    typename ConstructorMap::iterator it = types_.find(name);
    if (it == types_.end())
      return NULL;
    return (it->second)();
  }

  bool registerType(const std::string &name, Constructor c)
  {
    typename ConstructorMap::value_type value(name, c);
    types_.insert(value);
    return true;
  }

  void getTypes(std::vector<std::string> *result)
  {
    result->resize(types_.size());
    int i = 0;
    typename ConstructorMap::const_iterator it;
    for (it = types_.begin(); it != types_.end(); ++it)
      (*result)[i++] = it->first;
  }

private:
  Factory() {}
  ~Factory() {}

  typedef std::map<std::string,Constructor> ConstructorMap;
  ConstructorMap types_;
};




#endif
