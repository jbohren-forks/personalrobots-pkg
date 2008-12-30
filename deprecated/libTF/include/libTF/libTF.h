//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef LIBTF_HH
#define LIBTF_HH
#include <iostream>
#include <iomanip>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include <libTF/exception.h>
#include "libTF/Pose3DCache.h"
#include "boost/thread/mutex.hpp"

namespace libTF
{

/** ** Point ****
 *  \brief A simple point class incorperating the time and frameID
 * 
 * This is a point class designed to interact with libTF.  It 
 * incorperates the timestamp and associated frame to make 
 * association easier for the programmer.    
 */
 struct TFPoint 
{
  double x,y,z;
  uint64_t time;
  std::string frame;
 };

/** ** Point2D ****
 *  \brief A simple point class incorperating the time and frameID
 * 
 * This is a point class designed to interact with libTF.  It 
 * incorperates the timestamp and associated frame to make 
 * association easier for the programmer.    
 */
struct TFPoint2D
{
  double x,y;
  uint64_t time;
  std::string frame;
};


/** TFVector
 *  \brief A representation of a vector
 */
struct TFVector
{
  double x,y,z;
  uint64_t time;
  std::string frame;
};

/** TFVector2D
 *  \brief A representation of a 2D vector
 * 
 */
struct TFVector2D
{
  double x,y;
  uint64_t time;
  std::string frame;
};

/** TFEulerYPR
 * \brief A representation of Euler angles
 * Using Yaw, Pitch, Roll
 * commonly known as xyz Euler angles */
struct TFEulerYPR
{
  double yaw, pitch, roll;
  uint64_t time;
  std::string frame;
};

/** TFYaw
 * \brief Rotation about the Z axis.  
 */
struct TFYaw
{
  double yaw;
  uint64_t time;
  std::string frame;
};

/** TFPose
 *  \brief A representation of position in free space
 */
struct TFPose
{
  double x,y,z,yaw,pitch,roll;
  uint64_t time;
  std::string frame;
};

/** TFPose2D
 *  \brief A representation of 2D position
 */
struct TFPose2D
{
  double x,y,yaw;
  uint64_t time;
  std::string frame;
};


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

class TransformReference
{
public:
  /// All time within libTF is this format.
  typedef uint64_t ULLtime;
  
  /************* Constants ***********************/
  //  static const unsigned int MAX_NUM_FRAMES = 10000; //!< The max value of frameID (due to preallocation of pointers)
  static const unsigned int MAX_GRAPH_DEPTH = 100;   //!< The maximum number of time to recurse before assuming the tree has a loop.
  static const ULLtime DEFAULT_CACHE_TIME = 10 * 1000000000ULL;  //!< The default amount of time to cache data
  static const ULLtime DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0ULL; //!< The default amount of time to extrapolate


  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * \param max_extrapolation_distance How far to extrapolate before throwing an exception
   */
  TransformReference(bool interpolating = true, 
                     ULLtime cache_time = DEFAULT_CACHE_TIME,
                     uint64_t max_extrapolation_distance = DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  virtual ~TransformReference(void);

  /********** Mutators **************/
  /** \brief Set a new frame or update an old one.
   * \param frameid The destination frame
   * \param parentid The frame id of the parent frame.  
   * \param x Translation forward
   * \param y Translation left
   * \param z Translation up
   * \param yaw Rotation about Z
   * \param pitch Rotation about Y
   * \param roll Rotation about X
   * \param time The tiem at which to set the transform
   *
   * These euler angles are applied in the order top to bottom to the parent coordinate frame to 
   * transform it to the coordinate frame, frameid.  
   * 
   *  Possible exceptions are: TransformReference::LookupException
   */
  void setWithEulers(const std::string & frameid, const std::string & parentid, double x, double y, double z, double yaw, double pitch, double roll, ULLtime time) __attribute__((deprecated));

  /** \brief Set a transform using DH Parameters 
   * Conventions from http://en.wikipedia.org/wiki/Robotics_conventions 
   *  Possible exceptions are: TransformReference::LookupException
   */
  void setWithDH(const std::string & frameid, const std::string & parentid, double length, double alpha, double offset, double theta, ULLtime time) __attribute__((deprecated));

  /** \brief Set the transform using a matrix 
   *  Possible exceptions are: TransformReference::LookupException
   */
  void setWithMatrix(const std::string & frameid, const std::string & parentid, const NEWMAT::Matrix & matrix_in, ULLtime time) __attribute__((deprecated));
  /** \brief Set the transform using quaternions natively 
   *  Possible exceptions are: TransformReference::LookupException
   */
  void setWithQuaternion(const std::string & frameid, const std::string & parentid, double xt, double yt, double zt, double xr, double yr, double zr, double w, ULLtime time) __attribute__((deprecated));
  
  /** \brief Clear all data */
  void clear();


  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.  
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * 
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  NEWMAT::Matrix getMatrix(const std::string & target_frame, const std::string & source_frame, ULLtime time) __attribute__((deprecated));


  /** \brief Transform a point to a different frame */
  TFPoint transformPoint(const std::string & target_frame, const TFPoint & point_in) __attribute__((deprecated));
  /** \brief Transform a 2D point to a different frame */
  TFPoint2D transformPoint2D(const std::string & target_frame, const TFPoint2D & point_in) __attribute__((deprecated));
  /** \brief Transform a vector to a different frame */
  TFVector transformVector(const std::string & target_frame, const TFVector & vector_in) __attribute__((deprecated));
  /** \brief Transform a 2D vector to a different frame */
  TFVector2D transformVector2D(const std::string & target_frame, const TFVector2D & vector_in) __attribute__((deprecated));
  /** \brief Transform Euler angles between frames */
  TFEulerYPR transformEulerYPR(const std::string & target_frame, const TFEulerYPR & euler_in) __attribute__((deprecated));
  /** \brief Transform Yaw between frames. Useful for 2D navigation */
  TFYaw transformYaw(const std::string & target_frame, const TFYaw & euler_in) __attribute__((deprecated));
  /** \brief Transform a 6DOF pose.  (x, y, z, yaw, pitch, roll). */
  TFPose transformPose(const std::string & target_frame, const TFPose & pose_in) __attribute__((deprecated));
  /** \brief Transform a planar pose, x,y,yaw */
  TFPose2D transformPose2D(const std::string & target_frame, const TFPose2D & pose_in) __attribute__((deprecated));

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  std::string viewChain(const std::string & target_frame, const std::string & source_frame) __attribute__((deprecated));

  /** \brief A way to see what frames have been cached 
   * Useful for debugging 
   */
  std::string viewFrames() __attribute__((deprecated));


  /************ Possible Exceptions ****************************/

  /** \brief An exception class to notify of bad frame number 
   * 
   * This is an exception class to be thrown in the case that 
   * a frame not in the graph has been attempted to be accessed.
   * The most common reason for this is that the frame is not
   * being published, or a parent frame was not set correctly 
   * causing the tree to be broken.  
   */
  class LookupException : public libTF::Exception
  {
  public:
    LookupException(const std::string errorDescription) : libTF::Exception(errorDescription) { ; };
  };

  /** \brief An exception class to notify of no connection
   * 
   * This is an exception class to be thrown in the case 
   * that the Reference Frame tree is not connected between
   * the frames requested. */
  class ConnectivityException : public libTF::Exception
  {
  public:
    ConnectivityException(const std::string errorDescription) : libTF::Exception(errorDescription) { ; };
  };
  
  /** \brief An exception class to notify that the search for connectivity descended too deep. 
   * 
   * This is an exception class which will be thrown if the tree search 
   * recurses too many times.  This is to prevent the search from 
   * infinitely looping in the case that a tree was malformed and 
   * became cyclic.
   */
  class MaxDepthException : public libTF::Exception
  {
  public:
    MaxDepthException(const std::string errorDescription): libTF::Exception(errorDescription) { ; };
  };
  
  /** \brief An exception class to notify that the requested value would have required extrapolation, and extrapolation is not allowed.
   * 
   */
  class ExtrapolateException : public libTF::Exception
  { 
  public:
    ExtrapolateException(const std::string &errorDescription): libTF::Exception(errorDescription) { ; };
  };

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
  
  class RefFrame: public Pose3DCache 
    {
    public:

      /** Constructor */
      RefFrame(bool interpolating = true,  
               uint64_t  max_cache_time = DEFAULT_MAX_STORAGE_TIME,
               uint64_t  max_extrapolation_time = DEFAULT_MAX_EXTRAPOLATION_TIME); 
      
      /** \brief Get the parent nodeID */
      inline unsigned int getParent(){return parent_;};
      
      /** \brief Set the parent node 
       * return: false => change of parent, cleared history
       * return: true => no change of parent 
       * \param parentID The frameID of the parent
       */
      bool setParent(unsigned int parent_id);

    private:
      
      /** Internal storage of the parent */
      unsigned int parent_;

    };

  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  std::vector< RefFrame*> frames_;

  /** \brief A mutex to protect testing and allocating new frames */
  boost::mutex frame_mutex_;

  std::map<std::string, unsigned int> frameIDs_;
  std::vector<std::string> frameIDs_reverse;
  
  /// How long to cache transform history
  ULLtime cache_time;

  /// whether or not to interpolate or extrapolate
  bool interpolating;
  
  /// whether or not to allow extrapolation
  uint64_t max_extrapolation_distance;

 public:
  /** \brief An internal representation of transform chains
   * 
   * This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
  typedef struct 
  {
    std::vector<unsigned int> inverseTransforms;
    std::vector<unsigned int> forwardTransforms;
  } TransformLists;

 protected: 
  /************************* Internal Functions ****************************/
  
  /** \brief Add a new frame and parent
   * The frame transformatiosn are left unspecified.
   * \param frameid The id of a frame to add
   * \param parentid The id of the parent frame to the one being added
   *
   *  Possible exceptions are: TransformReference::InvaildFrame
   */
  void addFrame(unsigned int frameid, unsigned int parentid);

  /** \brief An accessor to get a frame, which will throw an exception if the frame is no there. 
   * \param frame_number The frameID of the desired Reference Frame
   * 
   * This is an internal function which will get the pointer to the frame associated with the frame id
   * Possible Exception: TransformReference::LookupException
   */
  RefFrame* getFrame(unsigned int frame_number);

  unsigned int lookup(const std::string& frameid_str){
    unsigned int retval = 0;
    frame_mutex_.lock();
    std::map<std::string, unsigned int>::iterator it = frameIDs_.find(frameid_str);
    if (it == frameIDs_.end())
    {
      retval = frames_.size();
      frameIDs_[frameid_str] = retval;
      frames_.push_back( new RefFrame(interpolating, cache_time, max_extrapolation_distance));
      frameIDs_reverse.push_back(frameid_str);
    }
    else
      retval = frameIDs_[frameid_str];
    frame_mutex_.unlock();
    return retval;
  };


  /** Find the list of connected frames necessary to connect two different frames */
  TransformLists  lookUpList(unsigned int target_frame, unsigned int source_frame);
  
  /** Compute the transform based on the list of frames */
  NEWMAT::Matrix computeTransformFromList(const TransformLists & list, ULLtime time);

};
}
#endif //LIBTF_HH
