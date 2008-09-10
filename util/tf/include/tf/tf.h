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
#include <rosthread/mutex.h>

namespace tf
{

  /** \brief An internal representation of transform chains
   * 
   * This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
  typedef struct 
  {
    std::vector<unsigned int> inverseTransforms;
    std::vector<unsigned int> forwardTransforms;
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
  static const uint64_t DEFAULT_CACHE_TIME = 10 * 1000000000ULL;  //!< The default amount of time to cache data
  static const uint64_t DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0; //!< The default amount of time to extrapolate


  /** Constructor 
   * \param interpolating Whether to interpolate, if this is false the closest value will be returned
   * \param cache_time How long to keep a history of transforms in nanoseconds
   * \param max_extrapolation_distance How far to extrapolate before throwing an exception
   */
  TransformReference(bool interpolating = true, 
                     ULLtime cache_time = DEFAULT_CACHE_TIME,
                     unsigned long long max_extrapolation_distance = DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  virtual ~TransformReference(void);

  void setTransform(const Stamped<Transform>& transform, const std::string& parent_id);

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
  void lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                       const uint64_t& time, Stamped<T>& transform);
  //time traveling version
  void lookupTransform(const std::string& target_frame, const uint64_t& target_time, 
                       const std::string& source_frame, const uint64_t& _source_time, 
                       const std::string& fixed_frame, Stamped<T>& transform);  


  template<typename T>
  void transformStamped(const std::string& target_frame, const Stamped<T>& stamped_in, Stamped<T>& stamped_out);

  template<typename T>
  void transformStamped(const std::string& target_frame, const uint64_t& _target_time,const std::string& fixed_frame, 
                        const Stamped<T>& stamped_in, Stamped<T>& stamped_out);

  /** \brief Debugging function that will print the spanning chain of transforms.
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
   * TransformReference::MaxDepthException
   */
  std::string chainAsString(const std::string & target_frame, const std::string & source_frame);

  /** \brief A way to see what frames have been cached 
   * Useful for debugging 
   */
  std::string allFramesAsString();


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
               unsigned long long  max_cache_time = DEFAULT_MAX_STORAGE_TIME,
               unsigned long long  max_extrapolation_time = DEFAULT_MAX_EXTRAPOLATION_TIME); 
      
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
  ros::thread::mutex frame_mutex_;

  std::map<std::string, unsigned int> frameIDs_;
  std::vector<std::string> frameIDs_reverse;
  
  /// How long to cache transform history
   cache_time;

  /// whether or not to interpolate or extrapolate
  bool interpolating;
  
  /// whether or not to allow extrapolation
  unsigned long long max_extrapolation_distance;

 public:

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
