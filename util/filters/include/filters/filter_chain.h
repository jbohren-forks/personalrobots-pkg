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

#ifndef FILTERS_FILTER_CHAIN_H_
#define FILTERS_FILTER_CHAIN_H_

#include "filters/filter_base.h"
#include <sstream>
#include <vector>
#include <tinyxml/tinyxml.h>
#include "boost/shared_ptr.hpp"

namespace filters
{

template <typename T>
class FilterChain
{
public:
  /** \brief Create the filter chain object */
  FilterChain():  configured_(false){};

  /** \brief Configure the filter chain 
   * This will call configure on all filters which have been added
   * as well as allocate the buffers*/
  bool configure(unsigned int size, TiXmlDocument doc)
  {
    /*************************** Parse the XML ***********************************/
    TiXmlElement *config = doc.RootElement();

    //Verify incoming xml for proper naming and structure    
    if (!config)
    {
      ROS_ERROR("The XML given to add could not be parsed.");
      return false;
    }
    if (config->ValueStr() != "filters" &&
        config->ValueStr() != "filter")
    {
      ROS_ERROR("The XML given to add must have either \"filter\" or \
  \"filters\" as the root tag");
      return false;
    }
    //Step into the filter list if necessary
    if (config->ValueStr() == "filters")
    {
      config = config->FirstChildElement("filter");
    }
    
    //Iterate over all filter in filters (may be just one)
    for (; config; config = config->NextSiblingElement("filter"))
    {
      if (!config->Attribute("type"))
      {
        ROS_ERROR("Could not add a filter because no type was given");
        return false;
      }
      else if (!config->Attribute("name"))
      {
        ROS_ERROR("Could not add a filter because no name was given");
        return false;
      }
      else
      {
        //Check for name collisions within the list itself.
        for (TiXmlElement *self_config = config->NextSiblingElement("filter"); self_config ; self_config = self_config->NextSiblingElement("filter"))
        {
          if (!strcmp(self_config->Attribute("name"), config->Attribute("name")))
          {
            ROS_ERROR("A self_filter with the name %s already exists", config->Attribute("name"));
            return false;
          }
        }
      }
    }




    /********************** Do the allocation *********************/
    buffer0_.resize(size);
    buffer1_.resize(size);

    bool result = true;    

       
    config = doc.RootElement();

    //Step into the filter list if necessary
    if (config->ValueStr() == "filters")
    {
     config = config->FirstChildElement("filter");
    }

    for (  ; config; config = config->NextSiblingElement("filter"))
    {

        
    std::stringstream constructor_string;
    constructor_string << config->Attribute("type") << typeid(T).name();
    try
    {
      boost::shared_ptr<filters::FilterBase<T> > p( filters::FilterFactory<T>::Instance().CreateObject(constructor_string.str()));
      printf("type: %s\n", p.get()->getType().c_str());
      result = result &&  p.get()->configure(size, config);    
      reference_pointers_.push_back(p);
      printf("Configured %s:%s filter at %p\n", config->Attribute("type"),
             config->Attribute("name"),  p.get());
   }
    catch (typename Loki::DefaultFactoryError<std::string, filters::FilterBase<T> >::Exception & ex)
    {
      std::stringstream ss;
      ss /*<< ex.what()*/ << " A Filter of type \"" << config->Attribute("type") << "\" cannot be constructed with string " <<constructor_string.str() ;
      throw std::runtime_error(ss.str());
    }
        
        
    
   
    }
    
    if (result == true)
    {
      configured_ = true;
    }
    return result;
  };


  /** \brief Clear all filters from this chain */
  bool clear() 
  {
    configured_ = false;
    reference_pointers_.clear();
    buffer0_.clear();
    buffer1_.clear();
    return true;
  };
  
  /** \brief process data through each of the filters added sequentially */
  bool update(const T& data_in, T& data_out);


  ~FilterChain()
  {
    clear();

  };

private:
  std::vector<boost::shared_ptr<filters::FilterBase<T> > > reference_pointers_;

  T buffer0_; ///<! A temporary intermediate buffer
  T buffer1_; ///<! A temporary intermediate buffer
  bool configured_; ///<! whether the system is configured  

};

template <typename T>
bool FilterChain<T>::update (const T& data_in, T& data_out)
{
  unsigned int list_size = reference_pointers_.size();
  bool result;
  if (list_size == 0)
  {
    data_out = data_in;
    result = true;
  }
  else if (list_size == 1)
    result = reference_pointers_[0]->update(data_in, data_out);
  else if (list_size == 2)
  {
    result = reference_pointers_[0]->update(data_in, buffer0_);
    result = result && reference_pointers_[1]->update(buffer0_, data_out);
  }
  else
  {
    result = reference_pointers_[0]->update(data_in, buffer0_);  //first copy in
    for (unsigned int i = 1; i <  reference_pointers_.size() - 1; i++) // all but first and last
    {
      if (i %2 == 1)
        result = result && reference_pointers_[i]->update(buffer0_, buffer1_);
      else
        result = result && reference_pointers_[i]->update(buffer1_, buffer0_);
      
    }
    if (list_size % 2 == 1) // odd number last deposit was in buffer0
      result = result && reference_pointers_.back()->update(buffer0_, data_out);
    else
      result = result && reference_pointers_.back()->update(buffer1_, data_out);
  }
  return result;
            
};


}



#endif //#ifndef FILTERS_FILTER_CHAIN_H_
