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

#ifndef FILTERS_MEDIAN_H
#define FILTERS_MEDIAN_H

#include <stdint.h>
#include <sstream>

#include <boost/scoped_ptr.hpp>

#include "filters/filter_base.h"

#include "filters/realtime_circular_buffer.h"


/*********************************************************************/
/*
 * Algorithm from N. Wirth's book, implementation by N. Devillard.
 * This code in public domain.
 */
#define ELEM_SWAP(a,b) { register elem_type t=(a);(a)=(b);(b)=t; }

namespace filters
{
/*---------------------------------------------------------------------------
  Function : kth_smallest()
  In : array of elements, # of elements in the array, rank k
  Out : one element
  Job : find the kth smallest element in the array
  Notice : use the median() macro defined below to get the median.
  Reference:
  Author: Wirth, Niklaus
  Title: Algorithms + data structures = programs
  Publisher: Englewood Cliffs: Prentice-Hall, 1976
  Physical description: 366 p.
  Series: Prentice-Hall Series in Automatic Computation
  ---------------------------------------------------------------------------*/
template <typename elem_type>
elem_type kth_smallest(elem_type a[], int n, int k)
{
  register int i,j,l,m ;
  register elem_type x ;
  l=0 ; m=n-1 ;
  while (l<m) {
    x=a[k] ;
    i=l ;
    j=m ;
    do {
      while (a[i]<x) i++ ;
      while (x<a[j]) j-- ;
      if (i<=j) {
        ELEM_SWAP(a[i],a[j]) ;
        i++ ; j-- ;
      }
    } while (i<=j) ;
    if (j<k) l=i ;
    if (k<i) m=j ;
  }
  return a[k] ;
}
#define median(a,n) kth_smallest(a,n,(((n)&1)?((n)/2):(((n)/2)-1)))
#undef ELEM_SWAP


/********************************************************************/

/** \brief A median filter which works on arrays.
 *
 */
class MedianDoubleFilter: public filters::FilterBase <double>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MedianDoubleFilter();

  /** \brief Destructor to clean up
   */
  ~MedianDoubleFilter();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in double array with length width
   * \param data_out double array with length width
   */
  virtual bool update(const double& data_in, double& data_out);
  virtual bool update(const std::vector<double>& data_in, std::vector<double>& data_out);
  
protected:
  std::vector<double> temp_storage_;                       ///< Preallocated storage for the list to sort
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<double> > > data_storage_;                       ///< Storage for data between updates
  
  std::vector<double> temp;  //used for preallocation and copying from non vector source


  uint32_t number_of_observations_;             ///< Number of observations over which to filter

};
}
#endif// FILTERS_MEDIAN_H
