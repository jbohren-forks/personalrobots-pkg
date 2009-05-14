/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
class TransformStamped
{
public:
  TransformStamped():
    frame_id("FRAME_ID_UNINITIALIZED"),
    parent_id("PARENT_ID_UNINITIALIZED"),
    _sec(0), _nsec(0) {};
  TransformStamped(const py::Transform& t, int sec, int nsec, const std::string& frame_id_in, const std::string& parent_id_in):
    transform(t),
    frame_id(frame_id_in),
    parent_id(parent_id_in), _sec(sec), _nsec(nsec)  {};
  TransformStamped(const TransformStamped& other)
  {
    transform = other.transform;
    frame_id = other.frame_id;
    parent_id = other.parent_id;
    _sec = other._sec;
    _nsec = other._nsec;
  };
  ~TransformStamped(){};
  inline bool operator==(const TransformStamped& other) const
  {
    if( _sec != other._sec) return false;
    else if ( _nsec != other._nsec) return false;
    else if ( frame_id != other.frame_id) return false;
    else if ( parent_id != other.parent_id) return false;
    else if (! (transform == other.transform)) return false;
    else return true;
  };
  py::Transform transform;
  std::string frame_id;
  std::string parent_id;
  int _sec;
  int _nsec;
};

/**\brief The swig interface data type */
class PoseStamped
{
public:
  PoseStamped():
    frame_id("FRAME_ID_UNINITIALIZED"),
    _sec(0), _nsec(0) {};
  PoseStamped(const py::Transform& p, int stamp_sec, int stamp_nsec, const std::string& frame_id_in):
    pose(p),
    frame_id(frame_id_in),
    _sec(stamp_sec), _nsec(stamp_nsec)  {};

  PoseStamped(const PoseStamped& other)
  {
    pose = other.pose;
    frame_id = other.frame_id;
    _sec = other._sec;
    _nsec = other._nsec;
  };
  ~PoseStamped(){};
  inline bool operator==(const PoseStamped& other) const
  {
    if( _sec != other._sec) return false;
    else if ( _nsec != other._nsec) return false;
    else if ( frame_id != other.frame_id) return false;
    else if (! (pose == (other.pose))) return false;
    else return true;
  };
  py::Transform pose;
  std::string frame_id;
  int _sec;
  int _nsec;
};

/**\brief The swig interface data type */
class PointStamped
{
public:
  PointStamped():
    frame_id("FRAME_ID_UNINITIALIZED"),
    _sec(0), _nsec(0) {};
  PointStamped(const py::Vector3& p, int sec, int nsec, const std::string& frame_id_in):
    point(p),
    frame_id(frame_id_in),
    _sec(sec), _nsec(nsec)  {};

  PointStamped(const PointStamped& other)
  {
    point = other.point;
    frame_id = other.frame_id;
    _sec = other._sec;
    _nsec = other._nsec;
  };

  ~PointStamped() {};
  inline bool operator==(const PointStamped& other) const
  {
    if( _sec != other._sec) return false;
    else if ( _nsec != other._nsec) return false;
    else if ( frame_id != other.frame_id) return false;
    else if (! (point == other.point)) return false; ///\todo failing one test
    else return true;
  };
  py::Vector3 point;
  std::string frame_id;
  int _sec;
  int _nsec;
};

/**\brief The swig interface data type */
class VectorStamped
{
public:
  VectorStamped():
    frame_id("FRAME_ID_UNINITIALIZED"),
    _sec(0), _nsec(0) {};
  VectorStamped(const py::Vector3& v, int sec, int nsec, const std::string& frame_id_in):
    vector(v),
    frame_id(frame_id_in),
    _sec(sec), _nsec(nsec)  {};

  VectorStamped(const VectorStamped& other)
  {
    vector = other.vector;
    frame_id = other.frame_id;
    _sec = other._sec;
    _nsec = other._nsec;
  };

  ~VectorStamped() { };
  inline bool operator==(const VectorStamped& other) const
  {
    if( _sec != other._sec) return false;
    else if ( _nsec != other._nsec) return false;
    else if ( frame_id != other.frame_id) return false;
    else if (! (vector == other.vector)) return false; ///\todo failing one test
    else return true;
  };
  py::Vector3 vector;
  std::string frame_id;
  int _sec;
  int _nsec;
};

/**\brief The swig interface data type */
class QuaternionStamped
{
public:
  QuaternionStamped():
    frame_id("FRAME_ID_UNINITIALIZED"),
    _sec(0), _nsec(0) {};
  QuaternionStamped(const py::Quaternion& q, int sec, int nsec, const std::string& frame_id_in):
    quaternion(q),
    frame_id(frame_id_in),
    _sec(sec), _nsec(nsec)  {};

  QuaternionStamped(const QuaternionStamped& other)
  {
    quaternion = other.quaternion;
    frame_id = other.frame_id;
    _sec = other._sec;
    _nsec = other._nsec;
  };

  ~QuaternionStamped(){};
  inline bool operator==(const QuaternionStamped& other) const
  {
    if( _sec != other._sec) return false;
    else if ( _nsec != other._nsec) return false;
    else if ( frame_id != other.frame_id) return false;
    else if (! (*quaternion == *(other.quaternion))) return false; ///\todo failing one test
    else return true;
  };
  py::Quaternion quaternion;
  std::string frame_id;
  int _sec;
  int _nsec;
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
  void setTransform(const TransformStamped& transform, const std::string& authority)
  {
    tf::Stamped<tf::Transform> tr;
    TransformStampedPytoBt(transform, tr);    
    tf_.setTransform(tr, authority);
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
  TransformStamped getTransform(const std::string& target_frame, const std::string& source_frame, 
                                double time)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, source_frame, ros::Time().fromSec(time), tr);
    TransformStamped retval;
    TransformStampedBttoPy(tr, retval);
    return retval;

  };
  //time traveling version
  TransformStamped getTransform(const std::string& target_frame, double target_time, 
                                const std::string& source_frame, double source_time, 
                                const std::string& fixed_frame)
  {
    tf::Stamped<tf::Transform> tr;
    tf_.lookupTransform(target_frame, ros::Time().fromSec(target_time),
                        source_frame, ros::Time().fromSec(source_time), 
                        fixed_frame, tr);
    TransformStamped retval;
    TransformStampedBttoPy(tr, retval);
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
  QuaternionStamped transformQuaternion(const std::string& target_frame, const QuaternionStamped& stamped_in) const
  {
    QuaternionStamped stamped_out;
    tf::Stamped<tf::Quaternion> temp_output, temp_input;
    QuaternionStampedPytoBt(stamped_in, temp_input);
    tf_.transformQuaternion(target_frame, 
                            temp_input,
                            temp_output);
    QuaternionStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };
  /** \brief Transform a Stamped Vector3 into the target frame */
  VectorStamped transformVector(const std::string& target_frame, const VectorStamped& stamped_in) const
  {
    VectorStamped stamped_out;
    tf::Stamped<tf::Vector3> temp_output, temp_input;
    VectorStampedPytoBt(stamped_in, temp_input);
    tf_.transformVector(target_frame, 
                        temp_input,
                        temp_output);
    VectorStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };
  /** \brief Transform a Stamped Point into the target frame */
  PointStamped transformPoint(const std::string& target_frame, const PointStamped& stamped_in) const
  {
    PointStamped stamped_out;
    tf::Stamped<tf::Point> temp_output, temp_input;
    PointStampedPytoBt(stamped_in, temp_input);
    tf_.transformPoint(target_frame, 
                       temp_input,
                       temp_output);
    PointStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };
  /** \brief Transform a Stamped Pose into the target frame */
  PoseStamped transformPose(const std::string& target_frame, const PoseStamped& stamped_in) const
  {
    PoseStamped stamped_out;
    tf::Stamped<tf::Pose> temp_output, temp_input;
    PoseStampedPytoBt(stamped_in, temp_input);
    tf_.transformPose(target_frame, 
                      temp_input,
                      temp_output);
    PoseStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };

  /** \brief Transform a Stamped Quaternion into the target frame */
  QuaternionStamped transformQuaternion(const std::string& target_frame, double target_time, 
                                        const QuaternionStamped& stamped_in, 
                                        const std::string& fixed_frame)  const
  {
    QuaternionStamped stamped_out;
    tf::Stamped<tf::Quaternion> temp_output, temp_input;
    QuaternionStampedPytoBt(stamped_in, temp_input);
    tf_.transformQuaternion(target_frame, ros::Time().fromSec(target_time),
                            temp_input, fixed_frame,
                            temp_output);
    QuaternionStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };
  /** \brief Transform a Stamped Vector3 into the target frame */
  VectorStamped transformVector(const std::string& target_frame, double target_time, 
                                const VectorStamped& stamped_in, 
                                const std::string& fixed_frame) const
  {
    VectorStamped stamped_out;
    tf::Stamped<tf::Vector3> temp_output, temp_input;
    VectorStampedPytoBt(stamped_in, temp_input);
    tf_.transformVector(target_frame, ros::Time().fromSec(target_time),
                        temp_input, fixed_frame,
                        temp_output);
    VectorStampedBttoPy(temp_output, stamped_out);    
    return stamped_out;
  };
  /** \brief Transform a Stamped Point into the target frame 
   * \todo document */
  PointStamped transformPoint(const std::string& target_frame, double target_time, 
                              const PointStamped& stamped_in, 
                              const std::string& fixed_frame) const
  {
    PointStamped stamped_out;
    tf::Stamped<tf::Point> temp_output, temp_input;
    PointStampedPytoBt(stamped_in, temp_input);
    tf_.transformPoint(target_frame, ros::Time().fromSec(target_time),
                       temp_input, fixed_frame,
                       temp_output);
    PointStampedBttoPy(temp_output, stamped_out);
    return stamped_out;    
  };
  /** \brief Transform a Stamped Pose into the target frame 
   * \todo document */
  PoseStamped transformPose(const std::string& target_frame, double target_time, 
                            const PoseStamped& stamped_in, 
                            const std::string& fixed_frame) const
  {
    PoseStamped stamped_out;
    tf::Stamped<tf::Pose> temp_output, temp_input;
    PoseStampedPytoBt(stamped_in, temp_input);
    tf_.transformPose(target_frame, ros::Time().fromSec(target_time),
                      temp_input, fixed_frame,
                      temp_output);
    PoseStampedBttoPy(temp_output, stamped_out);
    return stamped_out;
  };

private:
  tf::Transformer tf_;

  void QuaternionStampedPytoBt(const QuaternionStamped & in, tf::Stamped<tf::Quaternion>& out) const
  {
    const py::Quaternion & quat_in = in.quaternion;
    out = tf::Stamped<tf::Quaternion>(tf::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                                      ros::Time(in._sec, in._nsec),
                                      in.frame_id);
  };
  
  void QuaternionStampedBttoPy(const tf::Stamped<tf::Quaternion>& in, QuaternionStamped& out) const
  {
    const tf::Quaternion & quat_in = in;
    out = QuaternionStamped(py::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                            in.stamp_.sec, in.stamp_.nsec,
                            in.frame_id_);
  };

  void PointStampedPytoBt(const PointStamped & in, tf::Stamped<tf::Point>& out) const
  {
    const py::Vector3 & point_in = in.point;
    out = tf::Stamped<tf::Point>(tf::Point(point_in.x(), point_in.y(), point_in.z()),
                                 ros::Time(in._sec, in._nsec),
                                 in.frame_id);
  };
  
  void PointStampedBttoPy(const tf::Stamped<tf::Point>& in, PointStamped& out) const
  {
    const tf::Point & point_in = in;
    out = PointStamped(py::Vector3(point_in.x(), point_in.y(), point_in.z()),
                       in.stamp_.sec, in.stamp_.nsec,
                       in.frame_id_);
  };

  void VectorStampedPytoBt(const VectorStamped & in, tf::Stamped<tf::Vector3>& out) const
  {
    const py::Vector3 & vector_in = in.vector;
    out = tf::Stamped<tf::Vector3>(tf::Vector3(vector_in.x(), vector_in.y(), vector_in.z()),
                                   ros::Time(in._sec, in._nsec),
                                   in.frame_id);
  };
  
  void VectorStampedBttoPy(const tf::Stamped<tf::Vector3>& in, VectorStamped& out) const
  {
    const tf::Vector3 & vector_in = in;
    out = VectorStamped(py::Vector3(vector_in.x(), vector_in.y(), vector_in.z()),
                        in.stamp_.sec, in.stamp_.nsec,
                        in.frame_id_);
  };

  void PoseStampedPytoBt(const PoseStamped & in, tf::Stamped<tf::Pose>& out) const
  {
    const py::Vector3 & point_in = in.pose.getOrigin();
    const py::Quaternion & quat_in = in.pose.getRotation();
    out = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                                         tf::Point(point_in.x(), point_in.y(), point_in.z())),
                                ros::Time(ros::Time(in._sec, in._nsec)),
                                in.frame_id);
  };

  void PoseStampedBttoPy(const tf::Stamped<tf::Pose>& in, PoseStamped& out) const
  {
    const tf::Vector3 & orig = in.getOrigin();
    const tf::Quaternion & quat = in.getRotation();
    out = PoseStamped(py::Transform(py::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), 
                                    py::Vector3(orig.x(), orig.y(), orig.z())),
                      in.stamp_.sec, in.stamp_.nsec,
                      in.frame_id_);
  };

  void TransformStampedPytoBt(const TransformStamped & in, tf::Stamped<tf::Transform>& out) const
  {
    const py::Vector3 & point_in = in.transform.getOrigin();
    const py::Quaternion & quat_in = in.transform.getRotation();
    out = tf::Stamped<tf::Transform>(tf::Transform(tf::Quaternion(quat_in.x(), quat_in.y(), quat_in.z(), quat_in.w()),
                                                   tf::Point(point_in.x(), point_in.y(), point_in.z())),
                                     ros::Time(in._sec, in._nsec),
                                     in.frame_id, 
                                     in.parent_id);
  };

  void TransformStampedBttoPy(const tf::Stamped<tf::Transform>& in, TransformStamped& out) const
  {
    const tf::Vector3 & orig = in.getOrigin();
    const tf::Quaternion & quat = in.getRotation();
    out = TransformStamped(py::Transform(py::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()), 
                                         py::Vector3(orig.x(), orig.y(), orig.z())),
                           in.stamp_.sec, in.stamp_.nsec,
                           in.frame_id_,
                           in.parent_id_);
  };
};



#endif //PY_TF_H


