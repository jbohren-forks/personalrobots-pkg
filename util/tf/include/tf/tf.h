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

#ifndef TF_TF_H
#define TF_TF_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include <tf/exceptions.h>
#include "tf/time_cache.h"
#include <rosthread/mutex.h>


namespace tf
{
  /** \brief An internal representation of transform chains
   * 
   * This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
  typedef struct 
  {
    std::vector<Stamped<btTransform> > inverseTransforms;
    std::vector<Stamped<btTransform> > forwardTransforms;
  } TransformLists;

/** \brief A Class which provides coordinate transforms between any two frames in a system. 
 * 
 * This class provides a simple interface to allow recording and lookup of 
 * relationships between arbitrary frames of the system.
 * 
 * libTF assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.  
 * For example your typical robot would have a transform from global to real world.  And then from base to hand, and from base to head.  
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to hand.  
 * libTF is designed to take care of all the intermediate steps for you.  
 * 
 * Internal Representation 
 * libTF will store frames with the parameters necessary for generating the transform into that frame from it's parent and a reference to the parent frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.  
 * 
 * All function calls which pass frame ids can potentially throw the exception TransformReference::LookupException
 */
class Transformer
{
public:
  /************* Constants ***********************/
  static const unsigned int MAX_GRAPH_DEPTH = 100;   //!< The maximum number of time to recurse before assuming the tree has a loop.
  static const int64_t DEFAULT_CACHE_TIME = 10 * 1000000000ULL;  //!< The default amount of time to cache data
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0; //!< The default amount of time to extrapolate


  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * \param max_extrapolation_distance How far to extrapolate before throwing an exception
   */
  Transformer(bool interpolating = true, 
              ros::Duration cache_time_ = ros::Duration(DEFAULT_CACHE_TIME),
              ros::Duration max_extrapolation_distance_ = ros::Duration(DEFAULT_MAX_EXTRAPOLATION_DISTANCE));
  virtual ~Transformer(void);

  /** \brief Clear all data */
  void clear();

  void setTransform(const Stamped<btTransform>& transform);


  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.  
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * 
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  void lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                       ros::Time time, Stamped<btTransform>& transform);
  //time traveling version
  void lookupTransform(const std::string& target_frame, ros::Time target_time, 
                       const std::string& source_frame, ros::Time _source_time, 
                       const std::string& fixed_frame, Stamped<btTransform>& transform);  

  /** \brief Transform a Stamped data_in into data_out in traget frame */
  template<typename T>
  void transformStamped(const std::string& target_frame, const Stamped<T>& stamped_in, Stamped<T>& stamped_out);

  template<typename T>
  void transformMessage(const std::string& target_frame, const T& msg_in, T& msg_out);

  /** \brief Transform a Stamped data_in into data_out in traget frame at target time, using the given fixed frame
   * This is a bit complicated.  The net effect is that the data will be transformed to the fixed frame
   * at the time it is stamped with, and then transformed from the fixed frame to the target frame at the target time. 
   * \todo implement this. */
  template<typename T>
  void transformStamped(const std::string& target_frame, ros::Time _target_time,const std::string& fixed_frame, 
                        const Stamped<T>& stamped_in, Stamped<T>& stamped_out);


  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  std::string chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string & fixed_frame);

  /** \brief A way to see what frames have been cached 
   * Useful for debugging 
   */
  std::string allFramesAsString();

  /** \brief A way to get a std::vector of available frame ids */
  void getFrameStrings(std::vector<std::string>& ids);


protected:

  /** \brief The internal storage class for ReferenceTransform.  
   * 
   * An instance of this class is created for each frame in the system.
   * This class natively handles the relationship between frames.  
   *
   * The derived class Pose3DCache provides a buffered history of positions
   * with interpolation.
   * 
   */
  

  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  std::vector< TimeCache*> frames_;

  /** \brief A mutex to protect testing and allocating new frames */
  ros::thread::mutex frame_mutex_;

  std::map<std::string, unsigned int> frameIDs_;
  std::vector<std::string> frameIDs_reverse;
  
  /// How long to cache transform history
  ros::Duration cache_time;

  /// whether or not to interpolate or extrapolate
  bool interpolating;
  
  /// whether or not to allow extrapolation
  ros::Duration max_extrapolation_distance_;


  /************************* Internal Functions ****************************/

  /** \brief An accessor to get a frame, which will throw an exception if the frame is no there. 
   * \param frame_number The frameID of the desired Reference Frame
   * 
   * This is an internal function which will get the pointer to the frame associated with the frame id
   * Possible Exception: TransformReference::LookupException
   */
  TimeCache* getFrame(unsigned int frame_number);

  /// String to number for frame lookup with dynamic allocation of new frames
  unsigned int lookupFrameNumber(const std::string& frameid_str){
    unsigned int retval = 0;
    frame_mutex_.lock();
    std::map<std::string, unsigned int>::iterator map_it = frameIDs_.find(frameid_str);
    if (map_it == frameIDs_.end())
    {
      retval = frames_.size();
      frameIDs_[frameid_str] = retval;
      frames_.push_back( new TimeCache(interpolating, cache_time, max_extrapolation_distance_));
      frameIDs_reverse.push_back(frameid_str);
    }
    else
      retval = frameIDs_[frameid_str];
    frame_mutex_.unlock();
    return retval;
  };

  ///Number to string frame lookup may throw LookupException if number invalid
  std::string lookupFrameString(unsigned int frame_id_num)
  {
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw LookupException(ss.str());
    }
    else 
      return frameIDs_reverse[frame_id_num];

  };


  /** Find the list of connected frames necessary to connect two different frames */
  TransformLists  lookupLists(unsigned int target_frame, ros::Time target_time, unsigned int source_frame, ros::Time souce_time, unsigned int fixed_frame);
  
  /** Compute the transform based on the list of frames */
  btTransform computeTransformFromList(const TransformLists & list);

};

///\todo write out lots of these :-(

template<>
inline void Transformer::transformStamped<tf::Vector3>(const std::string& target_frame, 
                                                       const Stamped<tf::Vector3>& stamped_in, 
                                                       Stamped<tf::Vector3>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  btQuaternion quat = btQuaternion(stamped_in.data_.x(), stamped_in.data_.y(), stamped_in.data_.z()).normalized();
  double magnitude = sqrt(stamped_in.data_.x() * stamped_in.data_.x() + stamped_in.data_.y() * stamped_in.data_.y() + stamped_in.data_.z() * stamped_in.data_.z());
  quat = transform * quat;
  stamped_out.data_ = btVector3(quat.x() * magnitude, quat.y() * magnitude, quat.z() * magnitude);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};

template<typename T>
void Transformer::transformStamped(const std::string& target_frame, const Stamped<T>& stamped_in, Stamped<T>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  
  stamped_out.data_ = transform * stamped_in.data_;
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};




template<>
inline void Transformer::transformMessage<std_msgs::Vector3Stamped>(const std::string& target_frame, 
                                                             const std_msgs::Vector3Stamped& msg_in, 
                                                             std_msgs::Vector3Stamped& msg_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), msg_in.header.stamp, lookupFrameNumber( msg_in.header.frame_id), msg_in.header.stamp, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  btQuaternion quat = btQuaternion(msg_in.vector.x, msg_in.vector.y, msg_in.vector.z).normalized();
  double magnitude = sqrt(msg_in.vector.x * msg_in.vector.x + msg_in.vector.y * msg_in.vector.y + msg_in.vector.z * msg_in.vector.z);
  quat = transform * quat;
  msg_out.vector.x = quat.x() * magnitude;
  msg_out.vector.y = quat.y() * magnitude;
  msg_out.vector.z = quat.z() * magnitude;
  msg_out.header.stamp = msg_in.header.stamp;
  msg_out.header.frame_id = target_frame;
};



}
#endif //TF_TF_H
