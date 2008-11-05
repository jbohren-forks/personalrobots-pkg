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
  if ( frames_.size() > 1 )
  {
    for (std::vector< TimeCache*>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clearList();
    }
  }
  frame_mutex_.unlock();
}

void Transformer::setTransform(const Stamped<btTransform>& transform)
{
  getFrame(lookupFrameNumber(transform.frame_id_))->insertData(TransformStorage(transform, lookupFrameNumber(transform.parent_id_)));
  //  printf("adding data to %d \n", lookupFrameNumber(transform.frame_id_));
};


void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                     const ros::Time& time, Stamped<btTransform>& transform)
{
  TransformLists t_list = lookupLists(lookupFrameNumber( target_frame), time, lookupFrameNumber( source_frame));

  transform.setData( computeTransformFromList(t_list));
  transform.stamp_ = time;
  transform.frame_id_ = target_frame;

};

void Transformer::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame, Stamped<btTransform>& transform)
{
  //calculate first leg
  TransformLists t_list = lookupLists(lookupFrameNumber( fixed_frame), source_time, lookupFrameNumber( source_frame));
  btTransform temp1 = computeTransformFromList(t_list);


  TransformLists t_list2 = lookupLists(lookupFrameNumber( target_frame), target_time, lookupFrameNumber( fixed_frame));

  btTransform temp = computeTransformFromList(t_list2);

  transform.setData( temp1 * temp); ///\todo check order here
  transform.stamp_ = target_time;
  transform.frame_id_ = target_frame;

};

bool Transformer::getParent(const std::string& frame_id, ros::Time time, std::string& parent)
{

  tf::TimeCache* cache = getFrame(lookupFrameNumber(frame_id));
  TransformStorage temp;
  try
    {
      cache->getData(time, temp);
    }
  catch (tf::LookupException & ex) //No parent exists or frame doesn't exist
    {
      return false;
    }

  parent = temp.parent_id_;
  if (parent == "NO_PARENT")
    return false;
  return true;

};



TransformLists Transformer::lookupLists(unsigned int target_frame, ros::Time time, unsigned int source_frame)
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  ///\todo add fixed frame support

  TransformLists mTfLs;
  if (target_frame == source_frame)
    return mTfLs;  //Don't do anythign if we're not going anywhere

  TransformStorage temp;

  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  unsigned int last_inverse;
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
    throw LookupException("Frame didn't exist");
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);

      try{
        pointer->getData(time, temp);
      }
      catch (tf::LookupException & ex)
      {
        last_inverse = frame;
        // this is thrown when there is no data
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0) 
      {
        last_inverse = frame;
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
  unsigned int last_forward;
  if (getFrame(frame) == NULL) throw LookupException("fixme");; //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);


      try{
        pointer->getData(time, temp);
      }
      catch (tf::LookupException & ex)
      {
        last_forward = frame;
        //std::cout << ex.what() << " THROWN " << lookupFrameString(frame);
        // this is thrown when there is no data for the link
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0) 
      {
        last_forward = frame;
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
    if (mTfLs.forwardTransforms.size() == 0) //If it's going to itself it's already been caught
    {
      std::stringstream ss;
      ss<< "No Common ParentD between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
        << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    //    if (lookupFrameNumber(mTfLs.forwardTransforms.back().frame_id_) != target_frame)
    if (last_forward != source_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentC between " << lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
        << std::endl << allFramesAsString() << std::endl << mTfLs.forwardTransforms.size() << " forward length" 
        << " with " << lookupFrameString(last_forward) << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    else return mTfLs;
  }

  if (mTfLs.forwardTransforms.size() == 0)
  {
    if (mTfLs.inverseTransforms.size() == 0)  //If it's going to itself it's already been caught
    {
      std::stringstream ss;
      ss<< "No Common ParentB between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame) << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }

    if (lookupFrameNumber(mTfLs.inverseTransforms.back().parent_id_) != target_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentA between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl << mTfLs.inverseTransforms.back().parent_id_ << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    else return mTfLs;
  }


  /* Make sure the end of the search shares a parent. */
  if (last_forward != last_inverse)
  {
    std::stringstream ss;
    ss<< "No Common Parent, at top of search between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl;
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
        retTrans *= (lists.inverseTransforms[lists.inverseTransforms.size() -1 - i]); //Reverse to get left multiply
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
        retTrans = (lists.forwardTransforms[lists.forwardTransforms.size() -1 - i]).inverse() * retTrans; //Do this list backwards(from backwards) for it was generated traveling the wrong way
      }
      catch (tf::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.forwardTransforms[lists.forwardTransforms.size() -1 - i].frame_id_ << " is out of date. " << ex.what();
        throw ExtrapolationException(ss.str());
      }
    }

  return retTrans;
}


std::string Transformer::chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame)
{
  std::stringstream mstream;
  TransformLists lists = lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame));

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
      getFrame(counter)->getData(0ULL, temp);
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
      getFrame(counter)->getData(0ULL, temp);
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
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame,
                                  const Stamped<tf::Vector3>& stamped_in,
                                  Stamped<tf::Vector3>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  /** \todo may not be most efficient */
  btVector3 end = stamped_in;
  btVector3 origin = btVector3(0,0,0);
  btVector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  //  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};


void Transformer::transformQuaternion(const std::string& target_frame, const ros::Time& target_time,
                                      const Stamped<Quaternion>& stamped_in,
                                      const std::string& fixed_frame,
                                      Stamped<Quaternion>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame, const ros::Time& target_time,
                                  const Stamped<Vector3>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Vector3>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  /** \todo may not be most efficient */
  btVector3 end = stamped_in;
  btVector3 origin = btVector3(0,0,0);
  btVector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const ros::Time& target_time,
                                 const Stamped<Point>& stamped_in,
                                 const std::string& fixed_frame,
                                 Stamped<Point>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};

void Transformer::transformPose(const std::string& target_frame, const ros::Time& target_time,
                                const Stamped<Pose>& stamped_in,
                                const std::string& fixed_frame,
                                Stamped<Pose>& stamped_out)
{
  Stamped<Transform> transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = stamped_in.stamp_;
  stamped_out.frame_id_ = target_frame;
  //  stamped_out.parent_id_ = stamped_in.parent_id_;//only useful for transforms
};



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
