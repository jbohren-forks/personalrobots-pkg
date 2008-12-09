// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
#include "libTF/Pose3DCache.h"

#include <cassert>

using namespace libTF;

Pose3DCache::Pose3DCache(bool interpolating,
                           uint64_t max_cache_time,
                           uint64_t _max_extrapolation_time):
  max_storage_time(max_cache_time),
  max_length_linked_list(MAX_LENGTH_LINKED_LIST),
  max_extrapolation_time(_max_extrapolation_time),
  interpolating(interpolating)
{
  //Turn of caching, this should only keep a liked list of lenth 1
  // Thus returning only the latest
  //  if (!caching) max_length_linked_list = 1; //Removed, simply use 0 time to get first value

  pthread_mutex_init( &linked_list_mutex, NULL);
  //fixme Normalize();
  return;
};

Pose3DCache::~Pose3DCache()
{
  clearList();
};

void Pose3DCache::addFromQuaternion(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w, uint64_t time)
{
 Pose3DStorage temp;
 temp.xt = _xt; temp.yt = _yt; temp.zt = _zt; temp.xr = _xr; temp.yr = _yr; temp.zr = _zr; temp.w = _w; temp.time = time;

 add_value(temp);

} ;


void Pose3DCache::addFromMatrix(const NEWMAT::Matrix& matIn, uint64_t time)
{
  Pose3DStorage temp;
  temp.setFromMatrix(matIn);
  temp.time = time;

  add_value(temp);

};


void Pose3DCache::addFromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll, uint64_t time)
{
  Pose3DStorage temp;
  temp.setFromEuler(_x,_y,_z,_yaw,_pitch,_roll);
  temp.time = time;

  add_value(temp);
};

void Pose3DCache::addFromDH(double length, double alpha, double offset, double theta, uint64_t time)
{
  addFromMatrix(Pose3D::matrixFromDH(length, alpha, offset, theta),time);
};

Pose3DCache::Pose3DStorage& Pose3DCache::Pose3DStorage::operator=(const Pose3DStorage & input)
{
  xt = input.xt;
  yt = input.yt;
  zt = input.zt;
  xr = input.xr;
  yr = input.yr;
  zr = input.zr;
  w  = input.w ;
  time = input.time;

  return *this;
};


Pose3DCache::Pose3DStorage Pose3DCache::getPoseStorage(uint64_t time)
{
  Pose3DStorage temp;
  long long diff_time; //todo Find a way to use this offset. pass storage by reference and return diff_time??
  getValue(temp, time, diff_time);
  return temp;
};


NEWMAT::Matrix Pose3DCache::getMatrix(uint64_t time)
{
  Pose3DStorage temp;
  long long diff_time;
  getValue(temp, time, diff_time);

  //print Storage:
  //  std::cout << temp;

  return temp.asMatrix();
}

NEWMAT::Matrix Pose3DCache::getInverseMatrix(uint64_t time)
{
  return getMatrix(time).i();

};

void Pose3DCache::printMatrix(uint64_t time)
{
  std::cout << getMatrix(time);
};

void Pose3DCache::printStorage(const Pose3DStorage& storage)
{
  std::cout << storage;
};


bool Pose3DCache::getValue(Pose3DStorage& buff, uint64_t time, long long  &time_diff)
{
  Pose3DStorage p_temp_1;
  Pose3DStorage p_temp_2;
  //  long long temp_time;
  int num_nodes;

  bool retval = false;

  pthread_mutex_lock(&linked_list_mutex);
  try
  {
    num_nodes = findClosest(p_temp_1,p_temp_2, time, time_diff);
    if (num_nodes == 0)
      retval= false;
    else if (num_nodes == 1)
    {
      memcpy(&buff, &p_temp_1, sizeof(Pose3DStorage));
      retval = true;
    }
    else
    {
      if(interpolating)
	interpolate(p_temp_1, p_temp_2, time, buff);
      else
	buff = p_temp_1;
      retval = true;
    }
  }
  catch (ExtrapolationException &ex)
  {
    pthread_mutex_unlock(&linked_list_mutex);
    throw ex;
  }

  pthread_mutex_unlock(&linked_list_mutex);


  return retval;

};

void Pose3DCache::add_value(const Pose3DStorage &dataIn)
{
  pthread_mutex_lock(&linked_list_mutex);
  insertNode(dataIn);
  pruneList();
  pthread_mutex_unlock(&linked_list_mutex);
};


void Pose3DCache::insertNode(const Pose3DStorage & new_val)
{
  std::list<Pose3DStorage>::iterator it = storage_.begin();
  while(it != storage_.end())
  {
    if (it->time <= new_val.time)
      break;
    it++;
  }
  storage_.insert(it, new_val);
};

void Pose3DCache::pruneList()
{
  uint64_t current_time = storage_.begin()->time;

  while(!storage_.empty() && storage_.back().time + max_storage_time < current_time)
  {
    storage_.pop_back();
  }

};

void Pose3DCache::clearList()
{
  pthread_mutex_lock(&linked_list_mutex);
  storage_.clear();
  pthread_mutex_unlock(&linked_list_mutex);

};


int Pose3DCache::findClosest(Pose3DStorage& one, Pose3DStorage& two, const uint64_t target_time, long long &time_diff)
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
    time_diff = storage_.front().time; ///@todo what should this be?? difference from "now"?
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    one = *(storage_.begin());
    time_diff = target_time - storage_.begin()->time;
    return 1;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  std::list<Pose3DStorage>::iterator it = storage_.begin();
  while(it != storage_.end())
  {
    if (it->time <= target_time)
      break;
    it++;
  }
  //Catch the case it is the first value in the list
  if (it == storage_.begin())
  {
    one = *it;
    two = *(++it);
    time_diff = target_time - storage_.begin()->time;
    if ((unsigned int) time_diff > max_extrapolation_time) //Guarenteed in the future therefore positive
    {
      pthread_mutex_unlock(&linked_list_mutex);
      std::stringstream ss;
      ss << "Extrapolation Too Far in the future: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.time)/1000000000.0 << " and " << (two.time)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time)/1000000000.0 <<" at "<< (target_time - one.time)/1000000000.0<< " and " << (target_time - two.time)/1000000000.0 <<" respectively.";
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Catch the case where it's in the past
  if (it == storage_.end())
  {
    one = *(--it);
    two = *(--it);
    time_diff = target_time - one.time;
    if (time_diff < -(long long)max_extrapolation_time) //Guarenteed in the past
    {
      pthread_mutex_unlock(&linked_list_mutex);
      std::stringstream ss;
      ss << "Extrapolation Too Far in the past: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.time)/1000000000.0 << " and " << (two.time)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time)/1000000000.0 <<" at "<< (-target_time + one.time)/1000000000.0<< " and " << (-target_time + two.time)/1000000000.0 <<" respectively."; //sign flip since in the past
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = *(it); //Older
  two = *(--it); //Newer
  if (fabs(target_time - one.time) < fabs(target_time - two.time))
    time_diff = target_time - one.time;
  else
    time_diff = target_time - two.time;
  return 2;


};

// Quaternion slerp algorithm from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
void Pose3DCache::interpolate(const Pose3DStorage &one, const Pose3DStorage &two,uint64_t target_time, Pose3DStorage& output)
{
  output.time = target_time;

  // Calculate the ration of time betwen target and the two end points.
  long long total_diff = two.time - one.time;
  long long target_diff = target_time - one.time;

  //Check for zero distance case and just return
  if (abs(total_diff) < MIN_INTERPOLATION_DISTANCE || abs(target_diff) < MIN_INTERPOLATION_DISTANCE)
    {
      output = one;
      return;
    }

  double t = (double)target_diff / (double) total_diff; //ratio between first and 2nd position interms of time

  // Interpolate the translation
  output.xt = one.xt + t * (two.xt - one.xt);
  output.yt = one.yt + t * (two.yt - one.yt);
  output.zt = one.zt + t * (two.zt - one.zt);

  // Calculate angle between them.
  double cosHalfTheta = one.w * two.w + one.xr * two.xr + one.yr * two.yr + one.zr * two.zr;
  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (cosHalfTheta >= 1.0 || cosHalfTheta <= -1.0){
    output.w = one.w;output.xr = one.xr;output.yr = one.yr;output.zr = one.zr;
    return;
  }
  // Calculate temporary values.
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to qa or qb
  if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
    output.w = (one.w * 0.5 + two.w * 0.5);
    output.xr = (one.xr * 0.5 + two.xr * 0.5);
    output.yr = (one.yr * 0.5 + two.yr * 0.5);
    output.zr = (one.zr * 0.5 + two.zr * 0.5);
    return;
  }

  double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(t * halfTheta) / sinHalfTheta;
  //calculate Quaternion.
  output.w = (one.w * ratioA + two.w * ratioB);
  output.xr = (one.xr * ratioA + two.xr * ratioB);
  output.yr = (one.yr * ratioA + two.yr * ratioB);
  output.zr = (one.zr * ratioA + two.zr * ratioB);
  return;
};

double Pose3DCache::interpolateDouble(const double first, const uint64_t first_time, const double second, const uint64_t second_time, const uint64_t target_time)
{
  if ( first_time == second_time ) {
    return first;
  } else {
    return first + (second-first)* (double)((target_time - first_time)/(second_time - first_time));
  }
};
