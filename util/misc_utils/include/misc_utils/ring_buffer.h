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

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>
#include <vector>

/** \brief A ring buffer.
 *
 */
template <typename T>
class RingBuffer
{
public:
  /** \brief Construct a buffer of the correct length */
  RingBuffer(uint32_t number_of_elements);

  /** \brief Destructor to clean up
   */
  ~RingBuffer(){}
  
  T& operator[](int i);
  
  void push(T const &element);
  
protected:
  std::vector<T> buffer_;
  uint32_t length_;
  uint32_t buffer_ptr_;
};

template <typename T>
RingBuffer<T>::RingBuffer(uint32_t number_of_elements):
length_(number_of_elements),
buffer_ptr_(0)
{
  buffer_.resize(number_of_elements);
};

template <typename T>
T& RingBuffer<T>::operator[](int i)
{
  return buffer_[(buffer_ptr_+i)%length_];
}
template <typename T>
void RingBuffer<T>::push(T const &element)
{
  buffer_[buffer_ptr_]=element;
  buffer_ptr_=(buffer_ptr_+1)%length_;
}


#endif //#ifndef RING_BUFFER_H_
