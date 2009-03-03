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

#ifndef FILTERS_FILTER_BASE_H_
#define FILTERS_FILTER_BASE_H_

#include <tinyxml/tinyxml.h>
#include <typeinfo>
#include <loki/Factory.h>
#include "ros/assert.h"
#include "ros/console.h"

#include "boost/scoped_ptr.hpp"
#include <boost/algorithm/string.hpp>

namespace filters
{

typedef std::map<std::string, std::string> string_map_t;

template <typename T>
std::string getFilterID(const std::string & filter_name)
{
  return filter_name + typeid(T).name();
  
}




/** \brief A Base filter class to provide a standard interface for all filters
 *
 */
template<typename T>
class FilterBase
{
public:
  FilterBase():number_of_channels_(0), configured_(false){};
  virtual ~FilterBase(){};

  bool configure(unsigned int number_of_channels, TiXmlElement *config)
  {
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    number_of_channels_ = number_of_channels;
    bool retval = true;

    retval = retval && loadXml(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  };


  /** \brief Update the filter and return the data seperately
   * This is a lazy way to do this and can be overridden in the derived class
   */
  virtual bool update(const T& data_in, T& data_out)
  {
    std::vector<T> temp_in(1);
    std::vector<T> temp_out(1);
    temp_in[0] = data_in;
    bool retval =  update(temp_in, temp_out);
    data_out = temp_out[0];
    return retval;
  };
  /** \brief Update the filter and return the data seperately
   */
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)=0;

  std::string getType() {return typeid(T).name();};

  inline const std::string& getName(){return filter_name_;};


protected:

  bool getStringParam(const std::string& name, std::string& value, const std::string& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }
    value = it->second;
    return true;
  }

  bool getDoubleParam(const std::string&name, double& value, const double& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }
    value = atof(it->second.c_str());
    
  }
  bool getIntParam(const std::string&name, int& value, const int& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }
    value = atoi(it->second.c_str());
    return true;
  }
  bool getUIntParam(const std::string&name, unsigned int& value, const unsigned int& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }
    value = atoi(it->second.c_str());
    return true;
  }
  
  bool getDoubleVectorParam(const std::string&name, std::vector<double>& value, const std::vector<double>& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }

    value.clear();
    std::vector<std::string> pieces;
    
    boost::split( pieces, it->second, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
      if (pieces[i].size() > 0)
        value.push_back(atof(pieces[i].c_str()));
    
    return true;
  }

  bool getStringVectorParam(const std::string&name, std::vector<std::string>& value, const std::vector<std::string>& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }

    value.clear();
    std::vector<std::string> pieces;
    
    boost::split( pieces, it->second, boost::is_any_of(" "));
    value = pieces;
    return true;
  }
  


  virtual bool configure()=0;

  
  boost::scoped_ptr<TiXmlElement>  raw_xml_;
  std::string filter_name_;
  std::string filter_type_;
  unsigned int number_of_channels_;
  bool configured_;

  string_map_t params_;

private:
  bool setName(TiXmlElement * config)
  {  
    const char *name = config->Attribute("name");
    const char *type = config->Attribute("type");
    if (!name) 
    {
      ROS_ERROR("Filter didn't have name defined");
      return false;
    }
    if (!type) 
    {
      ROS_ERROR("Filter %s didn't have type defined", name);
      return false;
    }
    filter_name_ = std::string(name);
    filter_type_ = std::string(type);
    ROS_INFO("Configuring Filter of Type: %s with name %s", type, name);
    return true;
  };

  bool loadXml(TiXmlElement* config)
  {
    if (!config)
    {
      ROS_ERROR("Filter Configured w/o xml element");
      return false;
    }
    
    if (std::string(config->Value()) != std::string("filter"))
    {
      ROS_ERROR("Filter not being constructed with filter xml, type is %s", config->Value());
    }
    //Store a copy of the xml
    raw_xml_.reset(config->Clone()->ToElement());

    if (!setName(config))
    {
      ROS_ERROR("Filter Configured w/o name");
      return false;
    }

    TiXmlElement * params = config->FirstChildElement("params");
    if (params)
    {
      //Load params into map
      for (TiXmlAttribute * it = params->FirstAttribute(); it; it = it->Next())
      {
        ROS_DEBUG("Loading param %s with value %s\n", it->Name(), it->Value());
        params_[it->Name()] = it->Value();
      }; 
      
    }

    
    return true;    
  };
  
};


template <typename T>
class FilterFactory : public Loki::SingletonHolder < Loki::Factory< filters::FilterBase<T>, std::string >,
                                                     Loki::CreateUsingNew,
                                                     Loki::LongevityLifetime::DieAsSmallObjectChild >
{
  //empty
};
  


#define ROS_REGISTER_FILTER(c,t) \
  filters::FilterBase<t> * Filters_New_##c##__##t() {return new c< t >;}; \
  bool ROS_FILTER_## c ## _ ## t =                                                    \
    filters::FilterFactory<t>::Instance().Register(filters::getFilterID<t>(std::string(#c)), Filters_New_##c##__##t); 

}

#endif //#ifndef FILTERS_FILTER_BASE_H_
