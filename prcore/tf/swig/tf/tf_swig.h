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
#include "pybtTransform.h"


/**\brief The swig interface data type */
class StampedTransform
{
public:
  StampedTransform():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    stamp(0.0) {};
  py::Transform transform;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};



class pyTransformer
{
public:
  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * 
   */
  pyTransformer(bool interpolating = true, 
                ros::Duration cache_time = ros::Duration(10.0)):
    tf_(interpolating, cache_time){;};
  virtual ~pyTransformer(void){;};
  
  /** \brief Clear all data */
  void clear() {tf_.clear();};

  /** \brief Set a transform
   * Set a transform in the local library */
  void setTransform(const StampedTransform& transform)
  {
    const py::Quaternion& rot = transform.transform.getRotation();
    const py::Vector3& orig = transform.transform.getOrigin();
    tf_.setTransform(tf::Stamped<btTransform>(btTransform(btQuaternion(rot.getX(),
                                                                       rot.getX(),
                                                                       rot.getX(),
                                                                       rot.getX()),
                                                          btVector3(orig.getX(),
                                                                    orig.getX(),
                                                                    orig.getZ())),
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
  StampedTransform getTransform(const std::string& target_frame, const std::string& source_frame, 
                       double time)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, source_frame, ros::Time().fromSec(time), tr);
    StampedTransform retval;
    retval.transform.setOrigin(py::Vector3(tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z()));
    
    retval.transform.setRotation(py::Quaternion(tr.getRotation().x(), tr.getRotation().y(), tr.getRotation().z(), tr.getRotation().w()));

    retval.frame_id = tr.frame_id_;
    retval.parent_id = tr.parent_id_;
    retval.stamp = tr.stamp_.toSec();

    return retval;

  };
    //time traveling version
  StampedTransform getTransform(const std::string& target_frame, double target_time, 
                       const std::string& source_frame, double source_time, 
                       const std::string& fixed_frame)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, ros::Time().fromSec(target_time),
                        source_frame, ros::Time().fromSec(source_time), 
                        fixed_frame, tr);
    StampedTransform retval;
    retval.transform.setOrigin(py::Vector3(tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z()));
    
    retval.transform.setRotation(py::Quaternion(tr.getRotation().x(), tr.getRotation().y(), tr.getRotation().z(), tr.getRotation().w()));
    
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
  ros::Time getLatestCommonTime(const std::string& source, const std::string& dest)
  {
    ros::Time t;
    tf_.getLatestCommonTime(source, dest, t, NULL);
    return t;
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


