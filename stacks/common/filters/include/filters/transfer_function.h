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

#ifndef FILTERS_TRANSFER_FUNCTION_H
#define FILTERS_TRANSFER_FUNCTION_H
// Original version: Melonee Wise <mwise@willowgarage.com>

#include <stdint.h>
#include <math.h>
#include <vector>
#include <string>

#include <boost/scoped_ptr.hpp>

#include "filters/filter_base.h"
#include "filters/realtime_circular_buffer.h"

#include <pluginlib/plugin_macros.h>

namespace filters
{
/***************************************************/
/*! \class TransferFunctionDoubleFilter
    \brief One-dimensional digital filter class.

    This class calculates the output for \f$N\f$ one-dimensional
    digital filters. Where the input, \f$x\f$, is a (\f$N\f$ x 1) vector
    of inputs and the output, \f$y\f$, is a (\f$N\f$ x 1) vector of outputs.
    The filter is described by vectors \f$a\f$ and \f$b\f$ and
    implemented using the standard difference equation:<br>

    \f{eqnarray*}
    a[0]*y[n] = b[0]*x[n] &+& b[1]*x[n-1]+ ... + b[n_b]*x[n-n_b]\\
                          &-& a[1]*y[n-1]- ... - a[n_a]*y[n-n_a]
     \f}<br>


    If \f$a[0]\f$ is not equal to 1, the coefficients are normalized by \f$a[0]\f$.
    
    Example xml config:<br>

    <filter type="TransferFunctionDoubleFilter" name="filter_name"><br>
        <params a="1.0 0.5" b="0.2 0.2"/><br>
    </filter><br>

*/
/***************************************************/
class TransferFunctionDoubleFilter: public MultiChannelFilterBase <double>
{
public:
  /**
   * \brief Construct the filter 
   */
  TransferFunctionDoubleFilter() ;

  /** \brief Destructor to clean up
   */
  ~TransferFunctionDoubleFilter();
  
  /** \brief Configure the filter with the correct number of channels and params.
   * \param number_of_channels The number of inputs filtered.
   * \param config The xml that is parsed to configure the filter.
   */
  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in vector<T> with number_of_channels elements
   * \param data_out vector<double> with number_of_channels elements
   */
  virtual bool update(const double & data_in, double& data_out) ;
  virtual bool update(const std::vector<double> & data_in, std::vector<double>& data_out) ;
  
  

protected:
  
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<double> > > input_buffer_; //doublehe input sample history. 
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<double> > > output_buffer_; //The output sample history.
  
  std::vector<double>  temp; //used for storage and preallocation
  
  std::vector<double> a_;   //Transfer functon coefficients (output).
  std::vector<double> b_;   //Transfer functon coefficients (input).
  
};

}
#endif// FILTERS_TRANSFER_FUNCTION_H
