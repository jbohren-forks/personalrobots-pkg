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

#include "tf/tf.h"
#include <cassert>
#include <sys/time.h>
using namespace tf;

Transformer::Transformer(bool interpolating, 
                                uint64_t cache_time,
                                unsigned long long max_extrapolation_distance):
  cache_time(cache_time),
  interpolating (interpolating),
  max_extrapolation_distance(max_extrapolation_distance)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back( new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

Transformer::~Transformer()
{
  /* deallocate all frames */
  frame_mutex_.lock();
  for (std::vector<TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  {
    delete (*it);
  }
  frame_mutex_.unlock();
  
};


void Transformer::clear()
{
  frame_mutex_.lock();
  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  {
    (*it)->clearList();
  }
  frame_mutex_.unlock();
}



/*AT::Matrix Transformer::getMatrix(const std::string & target_frame, const std::string & source_frame, uint64_t time)
{
  unsigned int target_frame_num = lookup(target_frame);
  unsigned int source_frame_num = lookup(source_frame);
  NEWMAT::Matrix myMat(4,4);
  TransformLists lists = lookUpList(target_frame_num, source_frame_num);
  myMat = computeTransformFromList(lists,time);
  return myMat;
}
*/


/*void lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                     uint64_t time, Stamped<btTransform>& transform)
{
  lookupTransform(lookupFrameID( target_frame),lookupFrameID( source_frame), uint64_t time, Stamped<btTransform>& transform);
};
*/

//void Transformer::lookupTransform(unsigned int target_frame,unsigned int source_frame, uint64_t time, Stamped<btTransform>& transform)

TransformLists Transformer::lookupLists(unsigned int target_frame,unsigned int source_frame, uint64_t time)
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  TransformLists mTfLs;

  TransformStorage temp;
  
  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
    throw LookupException("Frame didn't exist");
  while (true)
    {

      TimeCache* pointer = getFrame(frame);
      if (pointer == NULL) break;

      pointer->getData(time, temp);
      mTfLs.inverseTransforms.push_back(TransformWithID(temp.stamp_, temp.data_));

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


      pointer->getData(time, temp);
      mTfLs.forwardTransforms.push_back(TransformWithID(temp.stamp_, temp.data_));

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
    if (mTfLs.forwardTransforms.back().frame_id != source_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentC" << std::endl << allFramesAsString() << std::endl;
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
    if (mTfLs.inverseTransforms.back().frame_id != target_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentA" << std::endl << allFramesAsString() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
  }
  
  /* Make sure the end of the search shares a parent. */
  if (mTfLs.inverseTransforms.back().frame_id != mTfLs.forwardTransforms.back().frame_id)
  {
    std::stringstream ss;
    ss<< "No Common Parent, at top of search" << std::endl << allFramesAsString() << std::endl;
    throw(ConnectivityException(ss.str()));
  }
  /* Make sure that we don't have a no parent at the top */
  if (mTfLs.inverseTransforms.back().frame_id == 0 ||  mTfLs.forwardTransforms.back().frame_id == 0)
    throw(ConnectivityException("NO_PARENT at top of tree"));
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */
  
  while (mTfLs.inverseTransforms.back().frame_id == mTfLs.forwardTransforms.back().frame_id)
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

btTransform Transformer::computeTransformFromList(const TransformLists & lists, uint64_t time)
{
  btTransform retTrans;
  
  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      try {
        retTrans *= (lists.inverseTransforms[lists.inverseTransforms.size() -1 - i]).transform; //Reverse to get left multiply
      }
      catch (tf::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.inverseTransforms[lists.inverseTransforms.size() -1 - i].frame_id << " is out of date. " << ex.what();
        throw ExtrapolationException(ss.str());
      }
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      try {
        retTrans *= (lists.forwardTransforms[i]).transform.inverse(); //Do this list backwards(from backwards) for it was generated traveling the wrong way
      }
      catch (tf::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.forwardTransforms[i].frame_id << " is out of date. " << ex.what();
        throw ExtrapolationException(ss.str());
      }
    }

  return retTrans;
}


std::string Transformer::chainAsString(const std::string & target_frame, const std::string & source_frame, uint64_t time)
{
  std::stringstream mstream;
  TransformLists lists = lookupLists(lookupFrameID(target_frame), lookupFrameID(source_frame), time);

  mstream << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      mstream << lists.inverseTransforms[i].frame_id<<", ";
    }
  mstream << std::endl;

  mstream << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      mstream << lists.forwardTransforms[i].frame_id<<", ";
    }
  mstream << std::endl;
  return mstream.str();
}

std::string Transformer::allFramesAsString()
{
  std::stringstream mstream;
  frame_mutex_.lock();
  
  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    mstream << "Frame "<< counter << " exists with parent " << /*frames_[counter]->getParent() <<*/ "." <<std::endl; /** \todo fixme */
  }
  frame_mutex_.unlock();
  return mstream.str();
}


tf::TimeCache* Transformer::getFrame(unsigned int frame_id) 
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else 
    return frames_[frame_id];

  /*  frame_mutex_.lock();
  std::map<unsigned int, TimeCache*>::const_iterator it = frames_.find(frame_id);
  bool found = it != frames_.end();
  TimeCache *frame = found ? it->second : NULL;
  frame_mutex_.unlock();
  
  if (!found){ 
    return NULL; // @todo check where HOBBLED THROW may effect
    std::stringstream ss; ss << "getFrame: Frame " << frame_id  << " does not exist."
                             << " Frames Present are: " <<std::endl << allFramesAsString() <<std::endl; 
    throw LookupException(ss.str());
  }
  return frame;
  */
};
