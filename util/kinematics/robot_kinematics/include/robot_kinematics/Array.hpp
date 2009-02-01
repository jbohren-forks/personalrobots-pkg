/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Array.hpp
 *
 *  Created on: Jan 4, 2009
 *      Author: Timothy Hunter <tjhunter@willowgarage.com>
 *
 *  FIXME move this inside Eigen
 */


#include <cassert>

#ifndef ARRAY_HPP_
#define ARRAY_HPP_

namespace Eigen
{
/**
 * @brief Storage type for Eigen Matrices. The size can be either fixed at compile-time or at runtime.
 */
template<typename T, int Size>
class StorageArray
{
public:
  explicit StorageArray(int size, const T & init = T());
  explicit StorageArray(const T & init = T());

  ~StorageArray(){}

  inline int length() const { return Size; }

  const T & operator[](int i) const { return array_[i]; }
  T & operator[](int i){ return array_[i]; }
private:
  T array_[Size];
};

template<typename T, int Size>
StorageArray<T,Size>::StorageArray(int size, const T & init)
{
  ei_assert(Size==size);
  for(int i=0;i<Size;++i)
    (*this)[i]=(init);
}

template<typename T, int Size>
StorageArray<T,Size>::StorageArray(const T & init)
{
  for(int i=0;i<Size;++i)
    (*this)[i]=(init);
}


template<typename T>
class StorageArray<T,Dynamic>
{
public:
  explicit StorageArray(int size, const T & init = T());
  ~StorageArray();

  inline int length() const { return size_; }

  const T & operator[](int i) const;
  T & operator[](int i);

private:
  int size_;
  T * array_;
};

template<typename T>
StorageArray<T,Dynamic>::StorageArray(int size, const T & init)
{
  size_=size;
  array_ = new T[size_];
  for(int i=0;i<size_;++i)
    (*this)[i]=(init);
}

template<typename T>
StorageArray<T,Dynamic>::~StorageArray()
{
  delete[] array_;
}

template<typename T>
T & StorageArray<T,Dynamic>::operator[](int i)
{
  assert(i<size_);
  return array_[i];
}

template<typename T>
const T & StorageArray<T,Dynamic>::operator[](int i) const
{
  assert(i<size_);
  return array_[i];
}

}
#endif /* ARRAY_HPP_ */
