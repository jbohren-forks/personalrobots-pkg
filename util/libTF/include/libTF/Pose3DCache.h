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
#ifndef POSE3DCACHE_HH
#define POSE3DCACHE_HH


#include <iostream>
#include <sstream>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <cmath>
#include <pthread.h>

#include "libTF/Pose3D.h"

namespace libTF{

  /** \brief A class to provide a linked list cache of Pose3D over time.
   *
   * This will maintain a linked list of Pose3D data types for max_cache_time.
   * It provides random access with a parameter of time.  It has protections 
   * on the max length of the linked list, the max storage time, the max extrapolation
   * time.  
   * 
   * This class will provide SLERP between the internal Quaternion representations 
   * of transformations.  It has a minimum interpolation distance which prevents interpolation if there
   * exists nearby data points, if computational time is an issue.  
   * 
   * time = 0 in accessors will produce the lastest value.  
   *
   * 
   *
   */
  class Pose3DCache {

  public:
    static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
    static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
    static const unsigned long long DEFAULT_MAX_STORAGE_TIME = 10ULL * 1000000000ULL; //!< default value of 10 seconds storage
    static const unsigned long long DEFAULT_MAX_EXTRAPOLATION_TIME = 10000000000ULL; //!< default max extrapolation of 10 seconds
    // Storage class
    class Pose3DStorage : public Pose3D
      {
      public:
        Pose3DStorage & operator=(const Pose3DStorage & input);
        unsigned long long time; //!<nanoseconds since 1970
      };

    /** \brief An exception class to notify that the requested value would have required extrapolation, and extrapolation is not allowed.
     * 
     */
    class ExtrapolateException : public std::exception
      { 
      public:
        ExtrapolateException(const std::string errorDescription) { 
          errorDescription_ = new std::string(errorDescription);
        };
        std::string * errorDescription_;
        ~ExtrapolateException() throw() { delete errorDescription_; };
        virtual const char* what() const throw()    { return errorDescription_->c_str(); };
    };
  
    /** \brief The constructor 
     * \param interpolating Whether or not to interpolating when accessing 
     * \param max_cache_time How long to cache past data (nanoseconds)
     * \param max_extrapolation_time How far to extrapolate before throwing an exception
     */
    Pose3DCache(bool interpolating = true, 
                unsigned long long  max_cache_time = DEFAULT_MAX_STORAGE_TIME,
                unsigned long long  max_extrapolation_time = DEFAULT_MAX_EXTRAPOLATION_TIME); 
    /** \brief Destructor */
    ~Pose3DCache();  

    /* Mutators */
    /** \brief Set the values with translation and quaternion notation */
    void addFromQuaternion(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w, unsigned long long time);
    /** \brief Set the values from a matrix */
    void addFromMatrix(const NEWMAT::Matrix& matIn, unsigned long long time);
    /** \brief  Set the values using translation and Euler angles */
    void addFromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll, unsigned long long time);
    /** \brief Set the values using DH Parameters */
    void addFromDH(double length, double alpha, double offset, double theta, unsigned long long time);

  
    /* Interpolated Accessors */
    /** \brief Return a Pose
     * \param time The desired time for the transformation */
    Pose3DStorage getPoseStorage(unsigned long long time);
    /** \brief Return the transform as a Matrix  
     * \param time The desired time for the transformation */
    NEWMAT::Matrix getMatrix(unsigned long long time);
    /** \brief Return the inverse matrix 
     * \param time The desired time for the transformation */
    NEWMAT::Matrix getInverseMatrix(unsigned long long time);
  
    /** \brief Print the stored value at a time as a matrix */
    void printMatrix(unsigned long long time);  //Not a critical part either
    /** \brief Print the value of a specific storage unit */
    void printStorage(const Pose3DStorage &storage); //Do i need this now that i've got the ostream method??

    /** \brief Remove all nodes from the list
     * This will clear all cached transformations.   */
    void clearList();
  
  
  private:
    /**** Linked List stuff ****/

    struct data_LL;
    /// The data structure for the linked list
    struct data_LL{
      Pose3DStorage data;
      struct data_LL * next;
      struct data_LL * previous;
    };
    /** \brief The internal method for getting data out of the linked list */
    bool getValue(Pose3DStorage& buff, unsigned long long time, long long  &time_diff);
    /** \brief The internal method for adding to the linked list 
     * which is by all the specific add functions.  */
    void add_value(const Pose3DStorage&);


    /** \brief insert a node into the sorted linked list   */
    void insertNode(const Pose3DStorage & );

    /** \brief prune data older than max_storage_time from the list   */
    void pruneList();

    /** \brief Find the closest two points in the list  
     *Return the distance to the closest one*/
    int findClosest(Pose3DStorage& one, Pose3DStorage& two, const unsigned long long target_time, long long &time_diff);

    /** \brief Interpolate between two nodes and return the interpolated value
     * This must always take two valid points!!!!!! */
    void interpolate(const Pose3DStorage &one, const Pose3DStorage &two, const unsigned long long target_time, Pose3DStorage& output);

    /** Linearly interpolate two doubles 
     *Used by interpolate to interpolate between double values */
    double interpolateDouble(const double, const unsigned long long, const double, const unsigned long long, const unsigned long long);

    ///How long to cache incoming values
    unsigned long long max_storage_time;
    ///Max length of linked list
    unsigned long long max_length_linked_list;
    ///Whether to allow extrapolation
    unsigned long long max_extrapolation_time;

    ///A mutex to prevent linked list collisions
    pthread_mutex_t linked_list_mutex;

    ///Pointer for the start of a sorted linked list.
    data_LL* first;
    ///Pointer for the end of a sorted linked list.
    data_LL* last;

    ///The length of the list
    unsigned int list_length;

    ///Whether or not to interpolate
    bool interpolating;

  };

};







#endif //POSE3DCACHE_HH
