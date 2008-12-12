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

#include "filters/median.h"
#include <cstring>
#include <stdio.h>

/*
 * Algorithm from N. Wirth's book, implementation by N. Devillard.
 * This code in public domain.
 */
typedef double elem_type ;
#define ELEM_SWAP(a,b) { register elem_type t=(a);(a)=(b);(b)=t; }
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



MedianFilter::MedianFilter(uint32_t number_of_observations, uint32_t elements_per_observation):
  last_updated_row_(0),
  iterations_(0),
  number_of_observations_(number_of_observations),
  elements_per_observation_(elements_per_observation)
{
  data_storage_ = new double[number_of_observations_ * elements_per_observation];
  temp_storage_ = new double[elements_per_observation];
};

MedianFilter::~MedianFilter()
{
  delete [] data_storage_;
  delete [] temp_storage_;
}

bool MedianFilter::update(double * data)
{
  return update (data, data);
}

bool MedianFilter::update(double const* const data_in, double* data_out)
{
  //update active row
  if (last_updated_row_ >= number_of_observations_ - 1)
    last_updated_row_ = 0;
  
  //copy incoming data into perminant storage
  memcpy(&data_storage_[elements_per_observation_ * last_updated_row_],
         &data_in, 
         sizeof(double) * elements_per_observation_);
  


  //Return values
  
  //keep track of number of rows used while starting up
  uint32_t length;
  if (iterations_ < number_of_observations_ )
  {
    iterations_++;
    length = iterations_;
  }
  else //all rows are allocated
  {
    length = number_of_observations_;
  }

  //Return each value
  for (uint32_t i = 0; i < number_of_observations_; i++)
  {
    printf("values: ");
    for (uint32_t row = 0; row < length; row ++)
    {
      temp_storage_[row] = data_storage_[i + row * elements_per_observation_];
      printf("%f, ", temp_storage_[row]);
    }
    data_out[i] = median(temp_storage_, length);
    printf("NET: %f\n", data_out[i]);
    
  }    
  
  return true;
}

