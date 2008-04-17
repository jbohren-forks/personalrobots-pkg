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
#ifndef QUATERNION3D_HH
#define QUATERNION3D_HH

#include <iostream>
#include <newmat/newmat.h>
#include <newmat/newmatio.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>

class Euler3D {
public:
  //Constructor
  Euler3D(double _x, double _y, double _z, double _yaw, double _pitch, double _roll);

  //Storage
  double  x,y,z,yaw,pitch,roll;
};


class Quaternion3D {

public:
  // Storage 
  class Quaternion3DStorage
  {
  public:

    /** Unary operators preserve timestamps
     * Binary operators destroy timestamps */

    // Utility functions to normalize and get magnitude.
    void Normalize();
    double getMagnitude();
    Quaternion3DStorage & operator=(const Quaternion3DStorage & input);


    /* Internal Data */    
    double xt, yt, zt, xr, yr, zr, w;
    unsigned long long time;
  };
  
  /** Constructors **/
  // Standard constructor
  Quaternion3D();
  
  /** Mutators **/
  // Set the values manually
  void Set(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w, unsigned long long time);

  //Set the values from a matrix
  void fromMatrix(const NEWMAT::Matrix& matIn, unsigned long long time);
  // Set the values using Euler angles
  void fromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll, unsigned long long time);
  // Set the values using DH Parameters
  void fromDH(double theta, double length, double distance, double alpha, unsigned long long time);

  
  /**** Utility Functions ****/
  static NEWMAT::Matrix matrixFromDH(double theta,
			      double length, double distance, double alpha);
  static NEWMAT::Matrix matrixFromEuler(double ax,
				 double ay, double az, double yaw,
				 double pitch, double roll);
  

  /** Interpolated Accessors **/
  // Return a Matrix
  Quaternion3D::Quaternion3DStorage asQuaternion(unsigned long long time);
  NEWMAT::Matrix asMatrix(unsigned long long time);

  //Print as a matrix
  void printMatrix(unsigned long long time);  //Not a critical part either
  void printStorage(const Quaternion3DStorage &storage); //Do i need this now that i've got the ostream method??

  // this is a function to return the current time in microseconds from the beginning of 1970
  static  unsigned long long Qgettime(void);

private:
  /**** Linked List stuff ****/
  static const long long MAX_STORAGE_TIME = 1000000000; // max of 100 seconds storage
 
  struct data_LL{
    Quaternion3DStorage data;
    data_LL * next;
    data_LL * previous;
  };

  bool getValue(Quaternion3DStorage& buff, unsigned long long time, long long  &time_diff);
  void add_value(const Quaternion3DStorage&);//todo fixme finish implementing this



  // insert a node into the sorted linked list
  void insertNode(const Quaternion3DStorage & );
  // prune data older than max_storage_time from the list
  void pruneList();

  //Find the closest two points in the list  
  //Return the distance to the closest one
  int findClosest(Quaternion3DStorage& one, Quaternion3DStorage& two, const unsigned long long target_time, long long &time_diff);

  //Interpolate between two nodes and return the interpolated value
  // This must always take two valid points!!!!!!
  // Only Cpose version implemented
  void interpolate(const Quaternion3DStorage &one, const Quaternion3DStorage &two, const unsigned long long target_time, Quaternion3DStorage& output);

  //Used by interpolate to interpolate between double values
  double interpolateDouble(const double, const unsigned long long, const double, const unsigned long long, const unsigned long long);

  //How long to cache incoming values
  unsigned long long max_storage_time;

  //A mutex to prevent linked list collisions
  pthread_mutex_t linked_list_mutex;

  //Pointers for the start and end of a sorted linked list.
  data_LL* first;
  data_LL* last;



};



//A global ostream overload for displaying storage
std::ostream & operator<<(std::ostream& mystream,const Quaternion3D::Quaternion3DStorage & storage);    








#endif //QUATERNION3D_HH
