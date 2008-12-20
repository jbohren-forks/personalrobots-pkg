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
 */
template <typename T>
class RingBuffer
{
public:

  /**
   * \brief Construct a buffer of the correct length
   * \param number_of_elements Defines the length of the ring buffer
   * \param default_elt Initializes each element of the ring buffer to this value. Defaults to T()
   */
  RingBuffer(uint32_t number_of_elements, const T &default_elt = T());

  /** \brief Destructor to clean up
   */
  ~RingBuffer(){}

  /**
   * \brief Grabs an element i from the buffer in constant time. This is realtime safe.
   * \param i Element to retrieve, with buffer[0] being the most recent
   *            and buffer[N-1] being the oldest
   * \return A reference to the element being retrieved
   */
  T& operator[](int i);

  /**
   * \brief Pushes an element onto the front of the ring buffer (at position buffer[0])
   *           such that element buffer[0] returns most recent data
   * Note the this operation is realtime safe, so long as the equals operator for type <T>
   * is also realtime safe.  For std::vector, if vector::size() is the same for both vectors,
   * then the equals operator is realtime safe.
   */
  void push(T const &element);

  /**
   * \brief Get the length of the ring buffer
   * \return The length of the ring buffer
   */
  unsigned int size() { return buffer_.size(); }

protected:
  std::vector<T> buffer_;
  uint32_t buffer_ptr_;  // Index of the most recently pushed element
};

template <typename T>
RingBuffer<T>::RingBuffer(uint32_t number_of_elements, const T &default_elt):
  buffer_(number_of_elements, default_elt),
  buffer_ptr_(0)
{
};

template <typename T>
T& RingBuffer<T>::operator[](int i)
{
  return buffer_[(buffer_ptr_ + i) % buffer_.size()];
}

template <typename T>
void RingBuffer<T>::push(T const &element)
{
  buffer_ptr_ = (buffer_ptr_ - 1 + buffer_.size()) % buffer_.size();
  buffer_[buffer_ptr_] = element;
}


#endif //#ifndef RING_BUFFER_H_
