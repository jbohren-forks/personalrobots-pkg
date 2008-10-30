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

/** \author Tully Foote */

#ifndef TF_TIME_CACHE_H
#define TF_TIME_CACHE_H

#include <list>
#include "rosthread/mutex.h"
#include "tf/transform_datatypes.h"
#include "tf/exceptions.h"

#include "LinearMath/btTransform.h"

#include <sstream>

namespace tf
{

/** \brief Storage for transforms and their parent */
class  TransformStorage : public Stamped<btTransform>
{
public:
  TransformStorage(){};
  TransformStorage(const Stamped<btTransform>& data, unsigned int parent_id): Stamped<btTransform>(data), parent_frame_id(parent_id){};
  unsigned int parent_frame_id;
};


/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_TIME = 10000000000LL; //!< default max extrapolation of 10 seconds


  TimeCache(bool interpolating = true, ros::Duration  max_storage_time = ros::Duration(DEFAULT_MAX_STORAGE_TIME),
            ros::Duration  max_extrapolation_time = ros::Duration(DEFAULT_MAX_EXTRAPOLATION_TIME)):
    interpolating_(interpolating),
    max_storage_time_(max_storage_time),
    max_extrapolation_time_(max_extrapolation_time)
    {};


  ros::Duration getData(ros::Time time, TransformStorage & data_out); //returns distance in time to nearest value

  void insertData(const TransformStorage& new_data)
    {
      storage_lock_.lock();
      std::list<TransformStorage >::iterator storage_it = storage_.begin();
      while(storage_it != storage_.end())
      {
        if (storage_it->stamp_ <= new_data.stamp_)
          break;
        storage_it++;
      }
      storage_.insert(storage_it, new_data);
      storage_lock_.unlock();

      pruneList();
    };


  void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);  //specific implementation for each T


  void clearList() { storage_lock_.lock(); storage_.clear(); storage_lock_.unlock();};

 private:
  std::list<TransformStorage > storage_;

  bool interpolating_;
  ros::Duration max_storage_time_;
  ros::Duration max_extrapolation_time_;

  ros::thread::mutex storage_lock_;

  /// A helper function for getData
  //Assumes storage is already locked for it
  std::string findClosest(TransformStorage& one, TransformStorage& two, ros::Time target_time, ros::Duration &time_diff, uint8_t& num_solutions);

  void pruneList()
    {

      storage_lock_.lock();
      ros::Time latest_time = storage_.begin()->stamp_;

      while(!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < latest_time)
      {
        storage_.pop_back();
      }

      storage_lock_.unlock();
    };



};


}
#endif // TF_TIME_CACHE_H
