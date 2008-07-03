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
                           unsigned long long max_cache_time,
                           unsigned long long _max_extrapolation_time):
  max_storage_time(max_cache_time),
  max_length_linked_list(MAX_LENGTH_LINKED_LIST),
  max_extrapolation_time(_max_extrapolation_time),
  first(NULL),
  last(NULL),
  list_length(0),
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

void Pose3DCache::addFromQuaternion(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w, unsigned long long time)
{
 Pose3DStorage temp;
 temp.xt = _xt; temp.yt = _yt; temp.zt = _zt; temp.xr = _xr; temp.yr = _yr; temp.zr = _zr; temp.w = _w; temp.time = time;

 add_value(temp);
 
} ;


void Pose3DCache::addFromMatrix(const NEWMAT::Matrix& matIn, unsigned long long time)
{
  Pose3DStorage temp;
  temp.setFromMatrix(matIn);
  temp.time = time;  

  add_value(temp);

};


void Pose3DCache::addFromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll, unsigned long long time)
{
  Pose3DStorage temp;
  temp.setFromEuler(_x,_y,_z,_yaw,_pitch,_roll);
  temp.time = time;

  add_value(temp);  
};

void Pose3DCache::addFromDH(double length, double alpha, double offset, double theta, unsigned long long time)
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



//Note not member function
std::ostream & libTF::operator<<(std::ostream& mystream, const Pose3DCache::Pose3DStorage& storage)
{

  mystream << "Storage: " << storage.xt <<", " << storage.yt <<", " << storage.zt<<", " << storage.xr<<", " << storage.yr <<", " << storage.zr <<", " << storage.w<<std::endl; 
  return mystream;
};

Pose3DCache::Pose3DStorage Pose3DCache::getPoseStorage(unsigned long long time)
{
  Pose3DStorage temp;
  long long diff_time; //todo Find a way to use this offset. pass storage by reference and return diff_time??
  getValue(temp, time, diff_time);
  return temp;
};


NEWMAT::Matrix Pose3DCache::getMatrix(unsigned long long time)
{
  Pose3DStorage temp;
  long long diff_time;
  getValue(temp, time, diff_time);

  //print Storage:
  //  std::cout << temp;
 
  return temp.asMatrix();
}  

NEWMAT::Matrix Pose3DCache::getInverseMatrix(unsigned long long time)
{
  return getMatrix(time).i();

};

void Pose3DCache::printMatrix(unsigned long long time)
{
  std::cout << getMatrix(time);
};

void Pose3DCache::printStorage(const Pose3DStorage& storage)
{
  std::cout << storage;
};


bool Pose3DCache::getValue(Pose3DStorage& buff, unsigned long long time, long long  &time_diff)
{
  Pose3DStorage p_temp_1;
  Pose3DStorage p_temp_2;
  //  long long temp_time;
  int num_nodes;

  bool retval = false;

  pthread_mutex_lock(&linked_list_mutex);
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
  data_LL* p_current;
  data_LL* p_old;

  //  cout << "Inserting Node " << new_val.time << endl;

  //Base case empty list
  if (first == NULL)
    {
      //cout << "Base case" << endl;
      first = new data_LL;
      assert(first);
      first->data = new_val;
      first->next = NULL;
      first->previous = NULL;
      last = first;
    }
  else 
    {
      //Increment through until at the end of the list or in the right spot
      p_current = first;
      while (p_current != NULL && p_current->data.time > new_val.time)
	{
	  //  cout << "passed beyond " << p_current->data.time << endl;
	  p_current = p_current->next;
	}
      
      //THis means we hit the end of the list so just append the node
      if (p_current == NULL)
	{
	  //cout << "Appending node to the end" << endl;
	  p_current = new data_LL;
	  assert (p_current);
	  p_current->data = new_val;
	  p_current->previous = last;
	  p_current->next = NULL;

	  last = p_current;
	}
      else
	{
	  
	  //  cout << "Found a place to put data into the list" << endl;
	  
	  // Insert the new node
	  // Record where the old first node was
	  p_old = p_current;
	  //Fill in the new node
	  p_current = new data_LL;
	  assert (p_current);
	  p_current->data = new_val;
	  p_current->next = p_old;
	  p_current->previous = p_old->previous;
	  
	  
	  //point the old to the new 
	  p_old->previous = p_current;
	  

	  //If at the top of the list make sure we're not 
	  if (p_current->previous == NULL)
	    first = p_current;
	  else
	    p_current->previous->next = p_current;
	}



    }

  // Record that we have increased the length of th elist
  list_length ++;
  
};

void Pose3DCache::pruneList()
{
  data_LL* p_current = last;

  //  cout << "Pruning List" << endl;

  //Empty Set
  if (last == NULL) return;

  unsigned long long current_time = first->data.time;;


  //While time stamps too old
  while (p_current->data.time + max_storage_time < current_time || list_length >= max_length_linked_list)
    {
      //      cout << "Age of node " << (double)(-p_current->data.time + current_time)/1000000.0 << endl;
     // Make sure that there's at least one element in the list
      if (p_current->previous != NULL)
	{
	  // Remove the last node
	  p_current->previous->next = NULL;
	  last = p_current->previous;
	  delete p_current;
	  p_current = last;
	  // std::cout << " Pruning Node" << list_length << std::endl;
	  list_length--;
	}
      else 
	break;

    }
  
};

void Pose3DCache::clearList()
{
  pthread_mutex_lock(&linked_list_mutex);  
  
  data_LL * p_current = first;
  data_LL * p_last = NULL;

  // Delete all nodes in list
  while (p_current != NULL)
    {
      p_last = p_current;
      p_current = p_current->next;
      delete p_last;
    }

  //Clean up pointers
  first = NULL;
  last = NULL;
  pthread_mutex_unlock(&linked_list_mutex);  
  
};


int Pose3DCache::findClosest(Pose3DStorage& one, Pose3DStorage& two, const unsigned long long target_time, long long &time_diff)
{

  data_LL* p_current = first;


  // Base case no list
  if (first == NULL)
    {
      return 0;
    }
  
  //Case one element list or latest value is wanted.  
  else if (first->next == NULL || target_time == 0)
    {
      one = first->data;
      time_diff = target_time - first->data.time;
      return 1;
    }
  
  else
    {
      //Two or more elements

      //Find the one that just exceeds the time or hits the end
      //and then take the previous one
      p_current = first->next; //Start on the 2nd element so if we fail we fall back to the first one
      //  cout << p_current->data.time << " vs " << target_time << endl;
      //while (p_current->next != NULL && p_current->next->data.time < target_time)
      while (p_current->next != NULL && p_current->data.time > target_time)
	{
	  //	  std::cout << "Skipping over " << p_current->data << endl;
	  p_current = p_current->next;
	}
      
      one = p_current->data;
      two = p_current->previous->data;
      
      
      // Test Extrapolation Distance
      if(target_time > one.time + max_extrapolation_time ||  //Future Case
	 target_time + max_extrapolation_time < two.time) // Previous Case
        {
          pthread_mutex_unlock(&linked_list_mutex);
          throw MaxExtrapolation;
        }
      
      return 2;
    }
};

// Quaternion slerp algorithm from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
void Pose3DCache::interpolate(const Pose3DStorage &one, const Pose3DStorage &two,unsigned long long target_time, Pose3DStorage& output)
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

double Pose3DCache::interpolateDouble(const double first, const unsigned long long first_time, const double second, const unsigned long long second_time, const unsigned long long target_time)
{
  if ( first_time == second_time ) {
    return first;
  } else {
    return first + (second-first)* (double)((target_time - first_time)/(second_time - first_time));
  }
};
