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

// Original version: Melonee Wise <mwise@willowgarage.com>

#ifndef IIR_FILTERS_H_
#define IIR_FILTERS_H_

#include <stdint.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "ros/node.h"

#include "urdf/parser.h"
#include "filters/transfer_function.h"
#include "filters/filter_base.h"
#include "iir_filters/Filter.h"

namespace iir_filters
{
/***************************************************/
/*! \class IIRFilter
    \brief One-dimensional IIR filter class.

    This class calculates the coefficients for the difference equation 
    shown below, using octave methods (i.e. butter, cheby1, etc.). Where 
    the input, \f$x\f$, is a (\f$N\f$ x 1) vector of inputs and the output, 
    \f$y\f$, is a (\f$N\f$ x 1) vector of outputs. The filter is described 
    by the arguments of the octave methods (i.e. octave command: 
    [b,a] = butter(n, Wc)) and implemented using the standard 
    difference equation:<br>

    \f{eqnarray*}
    a[0]*y[n] = b[0]*x[n] &+& b[1]*x[n-1]+ ... + b[n_b]*x[n-n_b]\\
                          &-& a[1]*y[n-1]- ... - a[n_a]*y[n-n_a]
     \f}<br>


    If \f$a[0]\f$ is not equal to 1, the coefficients are normalized by \f$a[0]\f$.
    
    
    Example xml config:<br>

    <filter type="IIRFilter" name="filter_name"><br>
        <params name="butter" args="2 .2 high"/><br>
    </filter><br>

*/
/***************************************************/
template <typename T>
class IIRFilter: public filters::FilterBase <T>
{
public:
  /**
   * \brief Construct the filter 
   */
  IIRFilter() ;

  /** \brief Destructor to clean up
   */
  ~IIRFilter();
  
  /** \brief Configure the filter
   * The correct number of channels and params have already been loaded.
   */
  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in vector<T> with n elements
   * \param data_out vector<T> with n elements
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T>& data_out) ;
  
  
protected:
  
  ros::Node* node_;
  std::vector<std::string> args_;   //Butterworth args
  std::string type_;
  filters::FilterBase<T > * tf_filter_;
};

FILTERS_REGISTER_FILTER(IIRFilter, double)
FILTERS_REGISTER_FILTER(IIRFilter, float)

template <typename T>
IIRFilter<T>::IIRFilter()
{
  tf_filter_ = new filters::TransferFunctionFilter<T > ();
  
  if ((node_ = ros::Node::instance()) == NULL) 
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv);
    node_ = new ros::Node("iir_filter");
  }
}

template <typename T>
IIRFilter<T>::~IIRFilter()
{
  if (tf_filter_) delete tf_filter_;

};

template <typename T>
bool IIRFilter<T>::configure()
{
  // Parse the params of the filter from the xml.

  std::string type;

  if (!this->getStringParam("name", type, type_))
  {
    ROS_ERROR("IIR filter Needs name args");
    return false;
  }


  if (!this->getStringVectorParam("args", args_, args_))
  {
    ROS_ERROR("IIR filter Needs param args");
    return false;
  }
  printf("args are \n");
  for (unsigned int i = 0; i < args_.size(); i++)
  {
    printf("iir args[%d] are %s\n", i, args_[i].c_str());
  }
  
  TiXmlDocument doc;
  TiXmlElement *transfer_function_config;
  iir_filters::Filter::Request  req;
  iir_filters::Filter::Response res;
  
  req.name = type;
  req.args = args_;
  if (ros::service::call("filter_coeffs", req, res))
  {
    std::string xml_str("<filter type=\"TransferFunctionFilter\" name=\"");
    xml_str.append(filters::FilterBase<T>::getName());
    xml_str.append("\"> <params a=\"");
    for(uint32_t i=0; i<res.a.size();i++)
    { 
      std::stringstream ss;
      ss << res.a[i];
      xml_str.append(ss.str());
      xml_str.append(" ");
    }
    xml_str.append("\" b=\"" );
    for(uint32_t i=0; i<res.b.size();i++)
    { 
      std::stringstream ss;
      ss << res.b[i];
      xml_str.append(ss.str());
      xml_str.append(" ");
    }
    xml_str.append("\"/></filter>" );
    
    //printf("%s\n",xml_str.c_str());
    doc.Parse(xml_str.c_str()); 
    transfer_function_config = doc.RootElement();
  }
  else
  {
    ROS_ERROR("IIRFilter, \"%s\", could not get filter coefficients to build filter.", filters::FilterBase<T>::getName().c_str());
    return false;
  }
  
  return tf_filter_->configure(this->number_of_channels_, transfer_function_config);
};


template <typename T>
bool IIRFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
   // Ensure the correct number of inputs
  if (data_in.size() != this->number_of_channels_ || data_out.size() != this->number_of_channels_ )  
  {
    ROS_ERROR("Number of channels is %d, but data_in.size() = %d and data_out.size() = %d.  They must match", this->number_of_channels_, data_in.size(), data_out.size());
    return false;
  }
     
  return tf_filter_->update(data_in, data_out);
}

}

#endif //#ifndef IIR_FILTERS_H_
