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

#include "tf/time_cache.h"

using namespace tf;

int64_t TimeCache::getData(uint64_t time, TransformStorage & data_out) //returns distance in time to nearest value
  //bool Pose3DCache::getValue(Pose3DStorage& buff, unsigned long long time, long long  &time_diff)
{
  TransformStorage p_temp_1, p_temp_2;

  int num_nodes;
  int64_t time_diff;
  storage_lock_.lock();
  try
  {
    num_nodes = findClosest(p_temp_1,p_temp_2, time, time_diff);
    if (num_nodes == 0)
    {
      throw LookupException("No Data for this link");
    }
    else if (num_nodes == 1)
    {
      data_out = p_temp_1;
    }
    else
    {
      if(interpolating_ && ( p_temp_1.parent_frame_id == p_temp_2.parent_frame_id) ) // if we're interpolating and haven't reparented
        interpolate(p_temp_1, p_temp_2, time, data_out);
      else
        data_out = p_temp_1;
        
        
    }
  }
  //  catch (TFException &ex)
  catch (...)
  {
    storage_lock_.unlock();
    throw;
  }
  storage_lock_.unlock();
    
  return time_diff;

};


uint8_t TimeCache::findClosest(TransformStorage& one, TransformStorage& two, uint64_t target_time, int64_t &time_diff)
{
  //No values stored
  if (storage_.empty())
  {
    return 0;
  }

  //If time == 0 return the latest
  if (target_time == 0)
  {
    one = storage_.front();
    time_diff = storage_.front().stamp_; ///@todo what should this be?? difference from "now"?
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    one = *(storage_.begin());
    time_diff = target_time - storage_.begin()->stamp_;
    return 1;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  std::list<TransformStorage >::iterator storage_it = storage_.begin();
  while(storage_it != storage_.end())
  {
    if (storage_it->stamp_ <= target_time)
      break;
    storage_it++;
  }
  //Catch the case it is the first value in the list
  if (storage_it == storage_.begin())
  {
    one = *storage_it;
    two = *(++storage_it);
    time_diff = target_time - storage_.begin()->stamp_;
    if ((unsigned int) time_diff > max_extrapolation_time_) //Guarenteed in the future therefore positive
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the future: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.stamp_)/1000000000.0 << " and " << (two.stamp_)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_)/1000000000.0 <<" at "<< (target_time - one.stamp_)/1000000000.0<< " and " << (target_time - two.stamp_)/1000000000.0 <<" respectively.";
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Catch the case where it's in the past
  if (storage_it == storage_.end())
  {
    one = *(--storage_it);
    two = *(--storage_it);
    time_diff = target_time - one.stamp_;
    if (time_diff < -(long long)max_extrapolation_time_) //Guarenteed in the past
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the past: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.stamp_)/1000000000.0 << " and " << (two.stamp_)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_)/1000000000.0 <<" at "<< (-target_time + one.stamp_)/1000000000.0<< " and " << (-target_time + two.stamp_)/1000000000.0 <<" respectively."; //sign flip since in the past
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = *(storage_it); //Older
  two = *(--storage_it); //Newer
  if (fabs(target_time - one.stamp_) < fabs(target_time - two.stamp_))
    time_diff = target_time - one.stamp_;
  else
    time_diff = target_time - two.stamp_;
  return 2;


};

void TimeCache::interpolate(const TransformStorage& one, const TransformStorage& two, uint64_t time, TransformStorage& output)
{ 
  //Calculate the ratio
  btScalar ratio = ((double) ((int64_t)time - (int64_t)one.stamp_)) / ((double) ((int64_t)two.stamp_ - (int64_t)one.stamp_));
  
  //Interpolate translation
  btVector3 v;
  v.setInterpolate3(one.data_.getOrigin(), two.data_.getOrigin(), ratio);
  output.data_.setOrigin(v);
  
  //Interpolate rotation
  btQuaternion q1,q2;
  one.data_.getBasis().getRotation(q1);
  two.data_.getBasis().getRotation(q2);
  output.data_.setRotation(slerp( q1, q2 , ratio));
  output.frame_id_ = one.frame_id_;
  output.parent_frame_id = one.parent_frame_id;
};

