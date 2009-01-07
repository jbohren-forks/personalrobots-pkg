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

#ifndef PY_TF_H
#define PY_TF_H

#include "tf/tf.h"

class pyTransform
{
public:
  double qx, qy,qz,qw;
  double x,y,z;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

class pyTransformer
{
public:
  /************* Constants ***********************/
  static const unsigned int MAX_GRAPH_DEPTH = 100UL;   //!< The maximum number of time to recurse before assuming the tree has a loop.
  static const int64_t DEFAULT_CACHE_TIME = 10ULL * 1000000000ULL;  //!< The default amount of time to cache data
  static const int64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL; //!< The default amount of time to extrapolate


  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * 
   */
  pyTransformer(bool interpolating = true, 
              int64_t cache_time = DEFAULT_CACHE_TIME):
    tf_(interpolating, cache_time){;};
  virtual ~pyTransformer(void){;};
  
  /** \brief Clear all data */
  void clear() {tf_.clear();};

  void setTransform(const pyTransform& transform)
  {
    tf_.setTransform(tf::Stamped<btTransform>(btTransform(btQuaternion(transform.qx,
                                                                       transform.qy, 
                                                                       transform.qz,
                                                                       transform.qw),
                                                          btVector3(transform.x,
                                                                    transform.y,
                                                                    transform.z)),
                                                          ros::Time().fromSec(transform.stamp),
                                                          transform.frame_id,
                                                          transform.parent_id));
  };


  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.  
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * 
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  pyTransform getTransform(const std::string& target_frame, const std::string& source_frame, 
                       double time)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, source_frame, ros::Time().fromSec(time), tr);
    pyTransform retval;
    retval.x = tr.getOrigin().x();
    retval.y = tr.getOrigin().y();
    retval.z = tr.getOrigin().z();

    retval.qx = tr.getRotation().x();
    retval.qy = tr.getRotation().y();
    retval.qz = tr.getRotation().z();
    retval.qw = tr.getRotation().w();

    retval.frame_id = tr.frame_id_;
    retval.parent_id = tr.parent_id_;
    retval.stamp = tr.stamp_.toSec();

    return retval;

  };
    //time traveling version
  pyTransform getTransform(const std::string& target_frame, double target_time, 
                       const std::string& source_frame, double source_time, 
                       const std::string& fixed_frame)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, ros::Time().fromSec(target_time),
                        source_frame, ros::Time().fromSec(source_time), 
                        fixed_frame, tr);
    pyTransform retval;
    retval.x = tr.getOrigin().x();
    retval.y = tr.getOrigin().y();
    retval.z = tr.getOrigin().z();
    
    retval.qx = tr.getRotation().x();
    retval.qy = tr.getRotation().y();
    retval.qz = tr.getRotation().z();
    retval.qw = tr.getRotation().w();
    
    retval.frame_id = tr.frame_id_;
    retval.parent_id = tr.parent_id_;
    retval.stamp = tr.stamp_.toSec();
    
    return retval;
  };


                         
  bool canTransform(const std::string& target_frame, const std::string& source_frame, 
                    double time)
  {
    return tf_.canTransform(target_frame, source_frame, ros::Time().fromSec(time));};
  //time traveling version
  bool canTransform(const std::string& target_frame, double target_time, 
                       const std::string& source_frame, double source_time, 
                       const std::string& fixed_frame)
  {
    return tf_.canTransform(target_frame, ros::Time().fromSec(target_time),
                        source_frame, ros::Time().fromSec(source_time),
                        fixed_frame);
  };
                       
  /**@brief Return the latest rostime which is common across the spanning set 
   * zero if fails to cross */
  int getLatestCommonTime(const std::string& source, const std::string& dest, double time)
  {
    return tf_.getLatestCommonTime(source, dest, ros::Time().fromSec(time));
  };


  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  std::string chainAsString(const std::string & target_frame, double target_time, const std::string & source_frame, double source_time, const std::string & fixed_frame)
  { 
    return tf_.chainAsString(target_frame, ros::Time().fromSec(target_time), source_frame, ros::Time().fromSec(source_time), fixed_frame);
  };

  /** \brief A way to see what frames have been cached 
   * Useful for debugging 
   */
  std::string allFramesAsString(){return tf_.allFramesAsString();};

  /** \brief A way to see what frames have been cached 
   * Useful for debugging 
   */
  std::string allFramesAsDot() {return tf_.allFramesAsDot();};

  /**@brief Set the distance which tf is allow to extrapolate
   * \param distance How far to extrapolate before throwing an exception
   * default is zero */
  void setExtrapolationLimit(const double seconds)
  {tf_.setExtrapolationLimit(ros::Duration().fromSec(seconds));};

private:
  tf::Transformer tf_;

};



#endif //PY_TF_H


