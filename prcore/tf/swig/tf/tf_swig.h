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
  StampedTransform(py::Transform t, double stamp_in, const std::string& frame_id_in, const std::string& parent_id_in):
    transform(t), frame_id(frame_id_in),
    parent_id(parent_id_in), stamp(stamp_in)  {};
  py::Transform transform;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

/**\brief The swig interface data type */
class StampedPose
{
public:
  StampedPose():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    stamp(0.0) {};
  StampedPose(py::Transform t, double stamp_in, const std::string& frame_id_in):
    transform(t), frame_id(frame_id_in),
    parent_id("PARENT_ID_UNINITIALIZED"), stamp(stamp_in)  {};
  py::Transform transform;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

/**\brief The swig interface data type */
class StampedPoint
{
public:
  StampedPoint():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    stamp(0.0) {};
  StampedPoint(py::Vector3 p, double stamp_in, const std::string& frame_id_in):
    point(p), frame_id(frame_id_in),
    parent_id("PARENT_ID_UNINITIALIZED"), stamp(stamp_in)  {};
  py::Vector3 point;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

/**\brief The swig interface data type */
class StampedVector
{
public:
  StampedVector():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    stamp(0.0) {};
  StampedVector(py::Vector3 p, double stamp_in, const std::string& frame_id_in):
    vector(p), frame_id(frame_id_in),
    parent_id("PARENT_ID_UNINITIALIZED"), stamp(stamp_in)  {};
  py::Vector3 vector;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

/**\brief The swig interface data type */
class StampedQuaternion
{
public:
  StampedQuaternion():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    stamp(0.0) {};
  StampedQuaternion(py::Quaternion q, double stamp_in, const std::string& frame_id_in):
    quaternion(q), frame_id(frame_id_in),
    parent_id("PARENT_ID_UNINITIALIZED"), stamp(stamp_in)  {};
  py::Quaternion quaternion;
  std::string frame_id;
  std::string parent_id;
  double stamp;
};

void StampedQuaternionPytoBt(const StampedQuaternion & in, tf::Stamped<tf::Quaternion>& out)
{
  const py::Quaternion & quat_in = in.quaternion;
  out = tf::Stamped<tf::Quaternion>(tf::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                                    ros::Time().fromSec(in.stamp),
                                    in.frame_id);
};
  
void StampedQuaternionBttoPy(const tf::Stamped<tf::Quaternion>& in, StampedQuaternion& out)
{
  const tf::Quaternion & quat_in = in;
  out = StampedQuaternion(py::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                          in.stamp_.toSec(),
                          in.frame_id_);
};

void StampedPointPytoBt(const StampedPoint & in, tf::Stamped<tf::Point>& out)
{
  const py::Vector3 & point_in = in.point;
  out = tf::Stamped<tf::Point>(tf::Point(point_in.x(), point_in.y(), point_in.z()),
                               ros::Time().fromSec(in.stamp),
                               in.frame_id);
};
  
void StampedPointBttoPy(const tf::Stamped<tf::Point>& in, StampedPoint& out)
{
  const tf::Point & point_in = in;
  out = StampedPoint(py::Vector3(point_in.x(), point_in.y(), point_in.z()),
                     in.stamp_.toSec(),
                     in.frame_id_);
};

void StampedVectorPytoBt(const StampedVector & in, tf::Stamped<tf::Vector3>& out)
{
  const py::Vector3 & vector_in = in.vector;
  out = tf::Stamped<tf::Vector3>(tf::Vector3(vector_in.x(), vector_in.y(), vector_in.z()),
                               ros::Time().fromSec(in.stamp),
                               in.frame_id);
};
  
void StampedVectorBttoPy(const tf::Stamped<tf::Vector3>& in, StampedVector& out)
{
  const tf::Vector3 & vector_in = in;
  out = StampedVector(py::Vector3(vector_in.x(), vector_in.y(), vector_in.z()),
                     in.stamp_.toSec(),
                     in.frame_id_);
};

class Transformer
{
public:
  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * 
   */
  Transformer(bool interpolating = true, 
              ros::Duration cache_time = ros::Duration(10.0)):
    tf_(interpolating, cache_time){;};
  virtual ~Transformer(void){;};
  
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
    const tf::Vector3& orig = tr.getOrigin();
    retval.transform.setOrigin(py::Vector3(orig.x(), orig.y(), orig.z()));
    
    const tf::Quaternion& rot = tr.getRotation();
    retval.transform.setRotation(py::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));

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

    const tf::Vector3& orig = tr.getOrigin();
    retval.transform.setOrigin(py::Vector3(orig.x(), orig.y(), orig.z()));
    
    const tf::Quaternion& rot = tr.getRotation();
    retval.transform.setRotation(py::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));

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
  {
    tf_.setExtrapolationLimit(ros::Duration().fromSec(seconds));
  };

  /** \brief Transform a Stamped Quaternion into the target frame */
  void transformQuaternion(const std::string& target_frame, const StampedQuaternion& stamped_in, StampedQuaternion& stamped_out) const
  {
    tf::Stamped<tf::Quaternion> temp_output, temp_input;
    StampedQuaternionPytoBt(stamped_in, temp_input);
    tf_.transformQuaternion(target_frame, 
                            temp_input,
                            temp_output);
    StampedQuaternionBttoPy(temp_output, stamped_out);
                            
  };
  /** \brief Transform a Stamped Vector3 into the target frame */
  //  void transformVector(const std::string& target_frame, const Stamped<tf::Vector3>& stamped_in, Stamped<tf::Vector3>& stamped_out) const;
  /** \brief Transform a Stamped Point into the target frame */
  //void transformPoint(const std::string& target_frame, const Stamped<tf::Point>& stamped_in, Stamped<tf::Point>& stamped_out) const;
  /** \brief Transform a Stamped Pose into the target frame */
  //void transformPose(const std::string& target_frame, const Stamped<tf::Pose>& stamped_in, Stamped<tf::Pose>& stamped_out) const;

  /** \brief Transform a Stamped Quaternion into the target frame 
      void transformQuaternion(const std::string& target_frame, const ros::Time& target_time, 
      const Stamped<tf::Quaternion>& stamped_in, 
      const std::string& fixed_frame, 
      Stamped<tf::Quaternion>& stamped_out) const;*/
  /** \brief Transform a Stamped Vector3 into the target frame 
      void transformVector(const std::string& target_frame, const ros::Time& target_time, 
      const Stamped<tf::Vector3>& stamped_in, 
      const std::string& fixed_frame, 
      Stamped<tf::Vector3>& stamped_out) const;*/
  /** \brief Transform a Stamped Point into the target frame 
   * \todo document
   void transformPoint(const std::string& target_frame, const ros::Time& target_time, 
   const Stamped<tf::Point>& stamped_in, 
   const std::string& fixed_frame, 
   Stamped<tf::Point>& stamped_out) const;*/
  /** \brief Transform a Stamped Pose into the target frame 
   * \todo document
   void transformPose(const std::string& target_frame, const ros::Time& target_time, 
   const Stamped<tf::Pose>& stamped_in, 
   const std::string& fixed_frame,
   Stamped<tf::Pose>& stamped_out) const;*/

private:
  tf::Transformer tf_;

};



#endif //PY_TF_H


