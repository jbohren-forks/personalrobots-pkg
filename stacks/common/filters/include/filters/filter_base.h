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
#include "ros/assert.h"
#include "ros/console.h"
#include "ros/ros.h"

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
  /** \brief Default constructor used by Filter Factories
   */
  FilterBase():configured_(false){};

  /** \brief Virtual Destructor
   */
  virtual ~FilterBase(){};

  /** \brief The public method to configure a filter from XML 
   * \param config The XmlRpcValue from which the filter should be initialized
   */
  bool configure(const XmlRpc::XmlRpcValue& config)
  {
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    bool retval = true;

    retval = retval && loadConfiguration(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  }

  /** \brief The public method to configure a filter from XML 
   * \param config The XML to initialize the filter with including name, type, and any parameters
   */
  bool configure(TiXmlElement *config)
  {
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    bool retval = true;

    retval = retval && loadXml(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  };


  /** \brief Update the filter and return the data seperately
   * This is an inefficient way to do this and can be overridden in the derived class
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   */
  virtual bool update(const T& data_in, T& data_out)=0;

  /** \brief Get the typeid of the Templated Datatype as a string */
  std::string getType() {return typeid(T).name();};

  /** \brief Get the name of the filter as a string */
  inline const std::string& getName(){return filter_name_;};


protected:

  /** \brief Pure virtual function for the sub class to configure the filter
   * This function must be implemented in the derived class.
   */
  virtual bool configure()=0;


  /** \brief Get a filter parameter as a string 
   * \param name The name of the parameter
   * \param value The string to set with the value
   * \param default_value The string to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
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

  /** \brief Get a filter parameter as a double
   * \param name The name of the parameter
   * \param value The double to set with the value
   * \param default_value The double to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
  bool getDoubleParam(const std::string&name, double& value, const double& default_value)
  {
    string_map_t::iterator it = params_.find(name);
    if (it == params_.end())
    {
      value = default_value;
      return false;
    }
    value = atof(it->second.c_str());
    return true;
  }

  /** \brief Get a filter parameter as a int
   * \param name The name of the parameter
   * \param value The int to set with the value
   * \param default_value The int to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
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


  /** \brief Get a filter parameter as a unsigned int
   * \param name The name of the parameter
   * \param value The unsignd int to set with the value
   * \param default_value The unsigned int to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
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
  

  /** \brief Get a filter parameter as a std::vector<double>
   * \param name The name of the parameter
   * \param value The std::vector<double> to set with the value
   * \param default_value The std::vector<double> to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
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

  /** \brief Get a filter parameter as a std::vector<string>
   * \param name The name of the parameter
   * \param value The std::vector<sgring> to set with the value
   * \param default_value The std::vector<string> to set the value to if the parameter of name doesn't exist
   * \return Whether or not the parameter of name existed */
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
  

  ///Storage for a pointer to the xml used to configure the filter
  ///This can be used by advanced filters using more than the param tags
  boost::scoped_ptr<TiXmlElement>  raw_xml_;
  ///The name of the filter
  std::string filter_name_;
  ///The type of the filter (Used by FilterChain for Factory construction)
  std::string filter_type_;
  /// Whether the filter has been configured.  
  bool configured_;

  ///Storage of the parsed xml parameters
  string_map_t params_;

private:
  /**\brief Set the name and type of the filter from the parameter server
   * \param param_name The parameter from which to read
   */
  bool setNameAndType(const XmlRpc::XmlRpcValue& config)
  {
    if(config.size() < 1)
    {
      ROS_ERROR("A filter must have both a name and a type defined");
      return false;
    }

    if(!config.hasMember("name"))
    {
      ROS_ERROR("Filter didn't have name defined, other strings are not allowed");
      return false;
    }

    std::string name = config["name"];

    if(!config.hasMember("type"))
    {
      ROS_ERROR("Filter %s didn't have type defined, other strings are not allowed", name.c_str());
      return false;
    }

    std::string type = config["type"];

    filter_name_ = name;
    filter_type_ = type;
    ROS_DEBUG("Configuring Filter of Type: %s with name %s", type.c_str(), name.c_str());
    return true;
  }

  /**\brief Set the name and type of the filter from XML 
   * \param config The XML from which to read
   */
  bool setNameAndType(TiXmlElement * config)
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
    ROS_DEBUG("Configuring Filter of Type: %s with name %s", type, name);
    return true;
  };

protected:
  bool loadConfiguration(const XmlRpc::XmlRpcValue& config)
  {
    if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("A filter configuration must be a map with fields name, type, and params");
      return;
    } 

    if (!setNameAndType(config))
    {
      return false;
    }

    //check to see if we have parameters in our list
    if(config.size() > 2 && config.hasMember("params"))
    {
      //get the params map
      XmlRpc::XmlRpcValue params = config["params"];

      if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("params must be a map");
        return false;
      }
      else if(config.size() > 2)
      {
        ROS_ERROR("The third element of the configuration list was not named params, not sure what to do with it so failing");
        return false;
      }
      else{
        //Load params into map
        for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
        {
          ROS_DEBUG("Loading param %s\n", it->first);
          params_[it->first] = it->second;
        } 
      }
    }

    return true;    
  }

  /** \brief Read in the XML and do basic configuration of FilterBase
   * \param config The XML to parse */
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

    if (!setNameAndType(config))
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
class MultiChannelFilterBase : public FilterBase<T>
{
  using FilterBase<T>::configured_;
  using FilterBase<T>::filter_type_;
  using FilterBase<T>::filter_name_;
public:
  MultiChannelFilterBase():number_of_channels_(0){};
  

  /** \brief The public method to configure a filter from XML 
   * \param number_of_channels How many parallel channels the filter will process
   * \param config The XML to initialize the filter with including name, type, and any parameters
   */
  bool configure(unsigned int number_of_channels, TiXmlElement *config)
  {
    if (configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filter_name_.c_str(), filter_type_.c_str());
    };
    configured_ = false;
    number_of_channels_ = number_of_channels;
    ROS_DEBUG("MultiChannelFilterBase configured with %d channels", number_of_channels_);
    bool retval = true;

    retval = retval && FilterBase<T>::loadXml(config);
    retval = retval && configure();
    configured_ = retval;
    return retval;
  };


  /** \brief A method to hide the base class method and warn if improperly called */
  bool configure(TiXmlElement *config)
  {
    ROS_ERROR("MultiChannelFilterBase configure should be called with a number of channels argument, assuming 1");
    return configure(1, config);
  }

  virtual bool configure()=0;
  

  /** \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)=0;

  virtual bool update(const T& data_in, T& data_out)
  {
    ROS_ERROR("THIS IS A MULTI FILTER DON'T CALL SINGLE FORM OF UPDATE");
    return false;
  };


protected:
  /// How many parallel inputs for which the filter is to be configured
  unsigned int number_of_channels_;
  

};

}
#endif //#ifndef FILTERS_FILTER_BASE_H_
