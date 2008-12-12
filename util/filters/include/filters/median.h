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

#ifndef FILTERS_MEDIAN_H_
#define FILTERS_MEDIAN_H_

#include <stdint.h>

#include "filters/filter_base.h"

/** \brief A median filter which works on double arrays.
 * 
 */
class MedianFilter: public FilterBase <double*>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MedianFilter(uint32_t number_of_observations, uint32_t elements_per_observation);

  /** \brief Destructor to clean up
   */
  ~MedianFilter();

  /** \brief Update filter mutating data in place
   * This will overwrite the results on top of the input
   * \param data This must be an array which is elements_per_observation long
   */
  bool update(double * data);


  /** \brief Update the filter and return the data seperately
   * \param data_in double array with length elements_per_observation
   * \param data_out double array with length elements_per_observation
   */
  bool update(const double* data_in, double* data_out);
  
protected:
  double * temp_storage_;                       ///< Preallocated storage for the list to sort

  double * data_storage_;                       ///< Storage for data between updates
  uint32_t last_updated_row_;                   ///< The last row to have been updated by the filter
  uint32_t iterations_;                         ///< Number of iterations up to number of observations

  uint32_t number_of_observations_;             ///< Number of observations over which to filter
  uint32_t elements_per_observation_;           ///< Number of elements per observation

};



#endif //#ifndef FILTERS_MEDIAN_H_
