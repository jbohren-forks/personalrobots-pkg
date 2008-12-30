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

#ifndef FILTERS_TRANSFER_FUNCTION_H_
#define FILTERS_TRANSFER_FUNCTION_H_

#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <vector>
#include "filters/filter_base.h"
#include "misc_utils/ring_buffer.h"

namespace filters
{
/***************************************************/
/*! \class TransferFunctionFilter
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

*/
/***************************************************/
template <typename T>
class TransferFunctionFilter: public FilterBase < std::vector<T> >
{
public:
  /**
   * \brief Construct the filter 
   * \param b Difference eq params for the input (See class description).
   * \param a Difference eq params for the output (See class description).
   * \param number_of_channels Defines the number of inputs filtered.
   */
  TransferFunctionFilter(std::vector<double> &b, std::vector<double> &a, unsigned int number_of_channels) ;

  /** \brief Destructor to clean up
   */
  ~TransferFunctionFilter()
  {
  }


  /** \brief Update the filter and return the data seperately
   * \param data_in vector<T> with n elements
   * \param data_out vector<T> with n elements
   */
  virtual bool update(std::vector<T> const * const data_in, std::vector<T>* data_out) ;

protected:
  unsigned int number_of_channels_;
  
  RingBuffer<std::vector<T> > input_buffer_;  
  RingBuffer<std::vector<T> > output_buffer_;
  
  std::vector<double> a_;   //Transfer functon coefficients (output)
  std::vector<double> b_;   //Transfer functon coefficients (input)


};

template <typename T>
TransferFunctionFilter<T>::TransferFunctionFilter(std::vector<double> &b, std::vector<double> &a, unsigned int number_of_channels):
  number_of_channels_(number_of_channels),
  input_buffer_(b.size(), std::vector<T>(number_of_channels, (T)0.0)),
  output_buffer_(a.size(), std::vector<T>(number_of_channels, (T)0.0)),
  a_(a), b_(b)
{
  // Prevents divide by zero.
  assert(a[0] != 0);  
  
  // Normalize the coeffs by a[0].
  if(a[0] != 1)
  { 
    for(uint32_t i = 0; i < b.size(); i++)
    {
      b_[i] = (b[i] / a[0]);
    }
    for(uint32_t i = 1; i < a.size(); i++)
    {
      a_[i] = (a[i] / a[0]);
    }
    a_[0] = (a[0] / a[0]);
  }
};


template <typename T>
bool TransferFunctionFilter<T>::update(std::vector<T> const* const data_in, std::vector<T>* data_out)
{
  // Ensure the correct number of inputs
  assert(data_in->size() == number_of_channels_);  
  
  // Copy data to prevent mutation if in and out are the same ptr
  std::vector<T> current_input = *data_in;        

  for (uint32_t i = 0; i < current_input.size(); i++)
  {
    (*data_out)[i]=b_[0] * current_input[i];

    for (uint32_t row = 0; row < input_buffer_.size(); row++)
    {
      (*data_out)[i] += b_[row+1] * input_buffer_[row][i];
    }
    for (uint32_t row = 0; row < output_buffer_.size(); row++)
    {
      (*data_out)[i] -= a_[row+1] * output_buffer_[row][i];
    }
  }
  input_buffer_.push(current_input);
  output_buffer_.push(*data_out);

  return true;
}

}

#endif //#ifndef FILTERS_TRANSFER_FUNCTION_H_
