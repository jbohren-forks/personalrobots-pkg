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

#include "tf/tf.h"
#include <cassert>
#include <sys/time.h>
using namespace tf;

Transformer::Transformer(bool interpolating, 
                                ros::Duration cache_time,
                                ros::Duration max_extrapolation_distance):
  cache_time(cache_time),
  interpolating (interpolating),
  max_extrapolation_distance_(max_extrapolation_distance)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(NULL);// new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

Transformer::~Transformer()
{
  /* deallocate all frames */
  frame_mutex_.lock();
  for (std::vector<TimeCache*>::iterator  cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    delete (*cache_it);
  }
  frame_mutex_.unlock();
  
};


void Transformer::clear()
{
  frame_mutex_.lock();
  for (std::vector< TimeCache*>::iterator  cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    (*cache_it)->clearList();
  }
  frame_mutex_.unlock();
}

void Transformer::setTransform(const Stamped<btTransform>& transform)
{
  getFrame(lookupFrameNumber(transform.frame_id_))->insertData(TransformStorage(transform, lookupFrameNumber(transform.parent_id_)));
  //  printf("adding data to %d \n", lookupFrameNumber(transform.frame_id_));
};
  

void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                     ros::Time time, Stamped<btTransform>& transform)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), time, lookupFrameNumber( source_frame), time, 0);

  transform.data_ = computeTransformFromList(t_list);
  transform.stamp_ = time;
  transform.frame_id_ = target_frame;

};

void Transformer::lookupTransform(const std::string& target_frame, ros::Time target_time, const std::string& source_frame, 
                     ros::Time source_time, const std::string& fixed_frame, Stamped<btTransform>& transform)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), target_time, lookupFrameNumber( source_frame), source_time, lookupFrameNumber(fixed_frame));

  transform.data_ = computeTransformFromList(t_list);
  transform.stamp_ = target_time;
  transform.frame_id_ = target_frame;

};




TransformLists Transformer::lookupLists(unsigned int target_frame, ros::Time target_time, unsigned int source_frame, ros::Time source_time, unsigned int fixed_frame)
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  ///\todo add fixed frame support

  TransformLists mTfLs;

  TransformStorage temp;
  
  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
    throw LookupException("Frame didn't exist");
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCache* pointer = getFrame(frame);
      if (pointer == NULL) break;

      try{
        pointer->getData(source_time, temp);
      }
      catch (tf::LookupException & ex)
      {
        // this is thrown when there is no data
        break;
      }
      mTfLs.inverseTransforms.push_back((Stamped<btTransform>)temp);

      frame = temp.parent_frame_id;


      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH){
        std::stringstream ss;
        ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl 
          << allFramesAsString() << std::endl;
        throw(LookupException(ss.str()));
      }
    }
  /*
    timeval tempt2;
  gettimeofday(&tempt2,NULL);
  std::cerr << "Side A " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  frame = target_frame;
  counter = 0;
  if (getFrame(frame) == NULL) throw LookupException("fixme");; //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      if (pointer == NULL) break;


      try{
        pointer->getData(target_time, temp);
      }
      catch (tf::LookupException & ex)
      {
        //        std::cout << ex.what() << " THROWN " << lookupFrameString(frame);
        // this is thrown when there is no data for the link
        break;
      }
      //      std::cout << "pushing back" << temp.frame_id_ << std::endl;
      mTfLs.forwardTransforms.push_back((Stamped<btTransform>)temp);

      frame = temp.parent_frame_id;

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH){
        std::stringstream ss;
        ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl 
          << allFramesAsString() << std::endl;
        throw(LookupException(ss.str()));
      }
    }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Side B " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */

  /* Check the zero length cases*/
  if (mTfLs.inverseTransforms.size() == 0)
  {
    if (mTfLs.forwardTransforms.size() == 0)
    {
      std::stringstream ss;
      ss<< "No Common ParentD" << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    if (lookupFrameNumber(mTfLs.forwardTransforms.back().frame_id_) != source_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentC forward.back ="<< mTfLs.forwardTransforms.back().frame_id_ << " but source frame =" 
        << frameIDs_reverse[source_frame]
        << std::endl << allFramesAsString() << std::endl << mTfLs.forwardTransforms.size() << " forward length" << std::endl;
      throw(ConnectivityException(ss.str()));
    }
  }
  
  if (mTfLs.forwardTransforms.size() == 0)
  {
    if (mTfLs.inverseTransforms.size() == 0)
    {
      std::stringstream ss;
      ss<< "No Common ParentB" << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    if (lookupFrameNumber(mTfLs.inverseTransforms.back().frame_id_) != target_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentA" << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
  }
  

  /* Make sure the end of the search shares a parent. */
  if (lookupFrameNumber(mTfLs.inverseTransforms.back().frame_id_) != lookupFrameNumber(mTfLs.forwardTransforms.back().frame_id_)) /// \todo rethink since the map is actually doing a string comparison inside
  {
    std::stringstream ss;
    ss<< "No Common Parent, at top of search inverse:"<<mTfLs.inverseTransforms.back().frame_id_ << " vs forward:" << mTfLs.forwardTransforms.back().frame_id_ << std::endl << allFramesAsString() << std::endl;
    throw(ConnectivityException(ss.str()));
  }
  /* Make sure that we don't have a no parent at the top */
  if (lookupFrameNumber(mTfLs.inverseTransforms.back().frame_id_) == 0 || lookupFrameNumber( mTfLs.forwardTransforms.back().frame_id_) == 0)
    throw(ConnectivityException("NO_PARENT at top of tree"));
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  
  while (lookupFrameNumber(mTfLs.inverseTransforms.back().frame_id_) == lookupFrameNumber(mTfLs.forwardTransforms.back().frame_id_))
  {
      mTfLs.inverseTransforms.pop_back();
      mTfLs.forwardTransforms.pop_back();

      // Make sure we don't go beyond the beginning of the list.  
      // (The while statement above doesn't fail if you hit the beginning of the list, 
      // which happens in the zero distance case.)
      if (mTfLs.inverseTransforms.size() == 0 || mTfLs.forwardTransforms.size() == 0)
	break;
    }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Done looking up list " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  return mTfLs;

}

btTransform Transformer::computeTransformFromList(const TransformLists & lists)
{
  btTransform retTrans;
  retTrans.setIdentity();
  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      try {
        retTrans *= (lists.inverseTransforms[lists.inverseTransforms.size() -1 - i]).data_; //Reverse to get left multiply
      }
      catch (tf::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.inverseTransforms[lists.inverseTransforms.size() -1 - i].frame_id_ << " is out of date. " << ex.what();
        throw ExtrapolationException(ss.str());
      }
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      try {
        ///\todo check whether r4608 applies here too
        retTrans *= (lists.forwardTransforms[i]).data_.inverse(); //Do this list backwards(from backwards) for it was generated traveling the wrong way
      }
      catch (tf::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.forwardTransforms[i].frame_id_ << " is out of date. " << ex.what();
        throw ExtrapolationException(ss.str());
      }
    }

  return retTrans;
}


std::string Transformer::chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame)
{
  std::stringstream mstream;
  TransformLists lists = lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame), source_time, lookupFrameNumber(fixed_frame));

  mstream << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      mstream << lists.inverseTransforms[i].frame_id_<<", ";
    }
  mstream << std::endl;

  mstream << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      mstream << lists.forwardTransforms[i].frame_id_<<", ";
    }
  mstream << std::endl;
  return mstream.str();
}

std::string Transformer::allFramesAsString()
{
  std::stringstream mstream;
  frame_mutex_.lock();

  TransformStorage temp;

  

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    unsigned int parent_id;
    try{
      getFrame(counter)->getData(0, temp);
      parent_id = temp.parent_frame_id;
    }
    catch (tf::LookupException& ex)
    {
      parent_id = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[parent_id] << "." <<std::endl;
  }
  frame_mutex_.unlock();
  return mstream.str();
}

void Transformer::getFrameStrings(std::vector<std::string> & vec)
{
  vec.clear();
  
  frame_mutex_.lock();

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    try{
      getFrame(counter)->getData(0, temp);
    }
    catch (tf::LookupException& ex)
    {
    }
    vec.push_back(frameIDs_reverse[counter]);
  }
  frame_mutex_.unlock();
  return;
}

tf::TimeCache* Transformer::getFrame(unsigned int frame_id) 
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else 
    return frames_[frame_id];
};


void Transformer::transformQuaternion(const std::string& target_frame, const Stamped<Quaternion>& stamped_in, Stamped<Quaternion>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  
  stamped_out.data_ = transform * stamped_in.data_;
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame, 
                                  const Stamped<tf::Vector3>& stamped_in, 
                                  Stamped<tf::Vector3>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  btTransform transform = computeTransformFromList(t_list);

  /** \todo may not be most efficient */
  btVector3 end = stamped_in.data_;
  btVector3 origin = btVector3(0,0,0);
  btVector3 output = (transform * end) - (transform * origin);
  stamped_out.data_ = output;

  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  
  stamped_out.data_ = transform * stamped_in.data_;
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), stamped_in.stamp_, lookupFrameNumber( stamped_in.frame_id_), stamped_in.stamp_, 0);
  
  btTransform transform = computeTransformFromList(t_list);
  
  stamped_out.data_ = transform * stamped_in.data_;
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  //  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformQuaternion(const std::string& target_frame, 
                                  const std_msgs::QuaternionStamped& msg_in, 
                                  std_msgs::QuaternionStamped& msg_out)
{
  Stamped<Quaternion> pin, pout;
  QuaternionStampedMsgToTF(msg_in, pin);
  transformQuaternion(target_frame, pin, pout);
  QuaternionStampedTFToMsg(pout, msg_out);
}

void Transformer::transformVector(const std::string& target_frame, 
                                  const std_msgs::Vector3Stamped& msg_in, 
                                  std_msgs::Vector3Stamped& msg_out)
{
  Stamped<Vector3> pin, pout;
  Vector3StampedMsgToTF(msg_in, pin);
  transformVector(target_frame, pin, pout);
  Vector3StampedTFToMsg(pout, msg_out);
}

void Transformer::transformPoint(const std::string& target_frame, 
                                  const std_msgs::PointStamped& msg_in, 
                                  std_msgs::PointStamped& msg_out)
{
  Stamped<Point> pin, pout;
  PointStampedMsgToTF(msg_in, pin);
  transformPoint(target_frame, pin, pout);
  PointStampedTFToMsg(pout, msg_out);
}

void Transformer::transformPose(const std::string& target_frame, 
                                  const std_msgs::PoseStamped& msg_in, 
                                  std_msgs::PoseStamped& msg_out)
{
  Stamped<Pose> pin, pout;
  PoseStampedMsgToTF(msg_in, pin);
  transformPose(target_frame, pin, pout);
  PoseStampedTFToMsg(pout, msg_out);
}

/*
void Transformer::transformTransform(const std::string& target_frame, 
                                  const std_msgs::TransformStamped& msg_in, 
                                  std_msgs::TransformStamped& msg_out)
{
  Stamped<Transform> pin, pout;
  TransformStampedMsgToTF(msg_in, pin);
  transformTransform(target_frame, pin, pout);
  TransformStampedTFToMsg(pout, msg_out);
}

*/
