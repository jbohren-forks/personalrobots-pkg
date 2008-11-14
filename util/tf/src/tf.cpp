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
                                ros::Duration cache_time):
  cache_time(cache_time),
  interpolating (interpolating),
  max_extrapolation_distance_(DEFAULT_MAX_EXTRAPOLATION_DISTANCE)
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
  std::string error_string;
  TransformLists t_list;
  ///\todo check return
  int retval = lookupLists(lookupFrameNumber( target_frame), time, lookupFrameNumber( source_frame), t_list, &error_string);

  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      throw LookupException(error_string);
    if (retval == CONNECTIVITY_ERROR)
      throw ConnectivityException(error_string);
  }
   
  if (test_extrapolation(time, t_list, &error_string))
    throw ExtrapolationException(error_string);

 
  transform.setData( computeTransformFromList(t_list));
  transform.stamp_ = time;
  transform.frame_id_ = target_frame;

};

void Transformer::lookupTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame, Stamped<btTransform>& transform)
{
  std::string error_string;
  //calculate first leg
  TransformLists t_list;
  ///\todo check return
  int retval = lookupLists(lookupFrameNumber( fixed_frame), source_time, lookupFrameNumber( source_frame), t_list, &error_string);
  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      throw LookupException(error_string);
    if (retval == CONNECTIVITY_ERROR)
      throw ConnectivityException(error_string);
  }
   
  if (test_extrapolation(target_time, t_list, &error_string))
    throw ExtrapolationException(error_string);

 
  btTransform temp1 = computeTransformFromList(t_list);


  TransformLists t_list2;
  ///\todo check return 
  retval =  lookupLists(lookupFrameNumber( target_frame), target_time, lookupFrameNumber( fixed_frame), t_list2, &error_string);
  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      throw LookupException(error_string);
    if (retval == CONNECTIVITY_ERROR)
      throw ConnectivityException(error_string);
  }
   
  if (test_extrapolation(target_time, t_list, &error_string))
    throw ExtrapolationException(error_string);

 

  btTransform temp = computeTransformFromList(t_list2);

  transform.setData( temp1 * temp); ///\todo check order here
  transform.stamp_ = target_time;
  transform.frame_id_ = target_frame;

};
bool Transformer::canTransform(const std::string& target_frame, const std::string& source_frame,
                     const ros::Time& time)
{
  TransformLists t_list;
  ///\todo check return
  int retval = lookupLists(lookupFrameNumber( target_frame), time, lookupFrameNumber( source_frame), t_list, NULL);

  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      return false;
    if (retval == CONNECTIVITY_ERROR)
      return false;
  }
   
  if (test_extrapolation(time, t_list, NULL))
    return false;

  return true;
};

bool Transformer::canTransform(const std::string& target_frame,const ros::Time& target_time, const std::string& source_frame,
                     const ros::Time& source_time, const std::string& fixed_frame)
{
  //calculate first leg
  TransformLists t_list;
  ///\todo check return
  int retval = lookupLists(lookupFrameNumber( fixed_frame), source_time, lookupFrameNumber( source_frame), t_list, NULL);
  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      return false;
    if (retval == CONNECTIVITY_ERROR)
      return false;
  }
   
  if (test_extrapolation(target_time, t_list, NULL))
    return false;

 
  btTransform temp1 = computeTransformFromList(t_list);


  TransformLists t_list2;
  ///\todo check return 
  retval =  lookupLists(lookupFrameNumber( target_frame), target_time, lookupFrameNumber( fixed_frame), t_list2, NULL);
  ///\todo WRITE HELPER FUNCITON TO RETHROW
  if (retval != NO_ERROR)
  {
    if (retval == LOOKUP_ERROR)
      return false;
    if (retval == CONNECTIVITY_ERROR)
      return false;
  }
   
  if (test_extrapolation(target_time, t_list, NULL))
    return false;

  return true;
};

bool Transformer::getParent(const std::string& frame_id, ros::Time time, std::string& parent)
{

  tf::TimeCache* cache = getFrame(lookupFrameNumber(frame_id));
  TransformStorage temp;
  if (! cache->getData(time, temp))     return false;
  if (temp.parent_id_ == "NO_PARENT")   return false;
  parent= temp.parent_id_;
  return true;

};

void Transformer::setExtrapolationLimit(const ros::Duration& distance)
{
  max_extrapolation_distance_ = distance;
}


int Transformer::lookupLists(unsigned int target_frame, ros::Time time, unsigned int source_frame, TransformLists& lists, std::string * error_string)
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  ///\todo add fixed frame support

  //Clear lists before operating
  lists.forwardTransforms.clear();
  lists.inverseTransforms.clear();
  //  TransformLists mTfLs;
  if (target_frame == source_frame)
    return 0;  //Don't do anythign if we're not going anywhere

  TransformStorage temp;

  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  unsigned int last_inverse;
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  {
    if (error_string) *error_string = "Source Frame Doesn't Exist";
    return LOOKUP_ERROR;//throw LookupException("Frame didn't exist");
  }
  while (true)
    {
      //      printf("getting data from %d:%s \n", frame, lookupFrameString(frame).c_str());

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);

      if (! pointer->getData(time, temp))
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
      lists.inverseTransforms.push_back(temp);

      frame = temp.parent_frame_id;


      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH)
      {
        if (error_string) 
        {
          std::stringstream ss;
          ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
            << allFramesAsString() << std::endl;
          *error_string =ss.str();
        }
        return LOOKUP_ERROR;
        //        throw(LookupException(ss.str()));
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
  if (getFrame(frame) == NULL) 
  {
    if (error_string) *error_string = "Target Frame Did Not Exist";
    return LOOKUP_ERROR;
  }//throw LookupException("fixme");; //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      ROS_ASSERT(pointer);


      if(!  pointer->getData(time, temp))
      {
        last_forward = frame;
        break;
      }

      //break if parent is NO_PARENT (0)
      if (frame == 0) 
      {
        last_forward = frame;
        break;
      }
      //      std::cout << "pushing back" << temp.frame_id_ << std::endl;
      lists.forwardTransforms.push_back(temp);
      frame = temp.parent_frame_id;

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH){
        if (error_string)
        {
          std::stringstream ss;
          ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
            << allFramesAsString() << std::endl;
          *error_string = ss.str();
        }
        return LOOKUP_ERROR;//throw(LookupException(ss.str()));
      }
    }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Side B " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */

  /* Check the zero length cases*/
  if (lists.inverseTransforms.size() == 0)
  {
    if (lists.forwardTransforms.size() == 0) //If it's going to itself it's already been caught
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<< "No Common ParentD between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
          << std::endl << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;//throw(ConnectivityException(ss.str()));
    }
    //    if (lookupFrameNumber(lists.forwardTransforms.back().frame_id_) != target_frame)
    if (last_forward != source_frame)
    {
      if (error_string) 
      {
        std::stringstream ss;
        ss<< "No Common ParentC between " << lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)
          << std::endl << allFramesAsString() << std::endl << lists.forwardTransforms.size() << " forward length" 
          << " with " << lookupFrameString(last_forward) << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;//throw(ConnectivityException(ss.str()));
    }
    else return 0;
  }

  if (lists.forwardTransforms.size() == 0)
  {
    if (lists.inverseTransforms.size() == 0)  //If it's going to itself it's already been caught
    {
      if (error_string) 
      {
        std::stringstream ss;
        ss<< "No Common ParentB between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame) << std::endl << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return CONNECTIVITY_ERROR;//throw(ConnectivityException(ss.str()));
    }

    if (lookupFrameNumber(lists.inverseTransforms.back().parent_id_) != target_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentA between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl << lists.inverseTransforms.back().parent_id_ << std::endl;
      if (error_string) *error_string = ss.str();
      return CONNECTIVITY_ERROR;//throw(ConnectivityException(ss.str()));
    }
    else return 0;
  }


  /* Make sure the end of the search shares a parent. */
  if (last_forward != last_inverse)
  {
    if (error_string) 
    {
      std::stringstream ss;
      ss<< "No Common Parent, at top of search between "<< lookupFrameString(target_frame) <<" and " << lookupFrameString(source_frame)  << std::endl << allFramesAsString() << std::endl;
      *error_string = ss.str();
    }
    return CONNECTIVITY_ERROR;//    throw(ConnectivityException(ss.str()));
  }
  /* Make sure that we don't have a no parent at the top */
  if (lookupFrameNumber(lists.inverseTransforms.back().frame_id_) == 0 || lookupFrameNumber( lists.forwardTransforms.back().frame_id_) == 0)
  {
    if (error_string) *error_string = "NO_PARENT at top of tree";
    return CONNECTIVITY_ERROR;//    throw(ConnectivityException("NO_PARENT at top of tree"));
  }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */

  while (lookupFrameNumber(lists.inverseTransforms.back().frame_id_) == lookupFrameNumber(lists.forwardTransforms.back().frame_id_))
  {
      lists.inverseTransforms.pop_back();
      lists.forwardTransforms.pop_back();

      // Make sure we don't go beyond the beginning of the list.
      // (The while statement above doesn't fail if you hit the beginning of the list,
      // which happens in the zero distance case.)
      if (lists.inverseTransforms.size() == 0 || lists.forwardTransforms.size() == 0)
	break;
  }
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Done looking up list " <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  return 0;

  }


bool Transformer::test_extrapolation(const ros::Time& target_time, const TransformLists& lists, std::string * error_string)
{
  bool retval = false;
  std::stringstream ss;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      if (lists.inverseTransforms[i].mode_ == EXTRAPOLATE_BACK)
      {
        if ( lists.inverseTransforms[i].stamp_ - target_time > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) {
            ss << "Extrapolation Too Far in the past: target_time = "<< (target_time).to_double() <<", closest data at "
               << lists.inverseTransforms[i].stamp_.toSec()  <<" which are farther away than max_extrapolation_distance "
               << (max_extrapolation_distance_).toSec() <<" at "<< (target_time - lists.inverseTransforms[i].stamp_).toSec()<<"."; //sign flip since in the past
          }
        }
      }
      else if( lists.inverseTransforms[i].mode_ == EXTRAPOLATE_FORWARD)
      {
        if ( target_time - lists.inverseTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) 
            ss << "Extrapolation Too Far in the future: target_time = "<< (target_time).toSec() <<", closest data at "
               << lists.inverseTransforms[i].stamp_.toSec()  <<" which are farther away than max_extrapolation_distance "
               << (max_extrapolation_distance_).toSec() <<" at "<< (target_time - lists.inverseTransforms[i].stamp_).toSec()<<"."; //sign flip since in the past
        }
      }
    }

  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      if (lists.forwardTransforms[i].mode_ == EXTRAPOLATE_BACK)
      {
        if ( lists.forwardTransforms[i].stamp_ - target_time > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) 
            ss << "Extrapolation Too Far in the past: target_time = "<< (target_time).toSec() <<", closest data at "
               << lists.forwardTransforms[i].stamp_.toSec()  <<" which are farther away than max_extrapolation_distance "
               << (max_extrapolation_distance_).toSec() <<" at "<< (target_time - lists.forwardTransforms[i].stamp_).toSec()<<"."; //sign flip since in the past
        }
      }
      else if( lists.forwardTransforms[i].mode_ == EXTRAPOLATE_FORWARD)
      {
        if (target_time - lists.forwardTransforms[i].stamp_ > max_extrapolation_distance_)
        {
          retval = true;
          if (error_string) 
            ss << "Extrapolation Too Far in the future: target_time = "<< (target_time).toSec() <<", closest data at "
               << lists.forwardTransforms[i].stamp_.toSec()  <<" which are farther away than max_extrapolation_distance "
               << (max_extrapolation_distance_).toSec() <<" at "<< (target_time - lists.forwardTransforms[i].stamp_).toSec()<<"."; //sign flip since in the past
        }
      }
    }

  if (error_string) *error_string = ss.str();
  return retval;


}


btTransform Transformer::computeTransformFromList(const TransformLists & lists)
{
  btTransform retTrans;
  retTrans.setIdentity();
  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      retTrans *= (lists.inverseTransforms[lists.inverseTransforms.size() -1 - i]); //Reverse to get left multiply
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      retTrans = (lists.forwardTransforms[lists.forwardTransforms.size() -1 - i]).inverse() * retTrans; //Do this list backwards(from backwards) for it was generated traveling the wrong way
    }

  return retTrans;
}


std::string Transformer::chainAsString(const std::string & target_frame, ros::Time target_time, const std::string & source_frame, ros::Time source_time, const std::string& fixed_frame)
{
  std::string error_string;
  std::stringstream mstream;
  TransformLists lists;
  ///\todo check return code 
  lookupLists(lookupFrameNumber(target_frame), target_time, lookupFrameNumber(source_frame), lists, &error_string);

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
    if(  getFrame(counter)->getData((uint64_t)0ULL, temp))
      parent_id = temp.parent_frame_id;
    else
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
    ///\todo why did I do this and then catch??
    /*   try{
      getFrame(counter)->getData((uint64_t)0ULL, temp);
    }
    catch (tf::LookupException& ex)
    {
    }
    */
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
