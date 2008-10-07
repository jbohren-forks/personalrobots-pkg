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

#include "libTF/libTF.h"
#include <cassert>
#include <sys/time.h>
using namespace libTF;

TransformReference::RefFrame::RefFrame(bool interpolating,
                                       uint64_t max_cache_time,
                                       uint64_t max_extrapolation_distance) :
  Pose3DCache(interpolating, max_cache_time, max_extrapolation_distance),
  parent_(0)
{
  return;
}


TransformReference::TransformReference(bool interpolating,
                                       ULLtime cache_time,
                                       uint64_t max_extrapolation_distance):
  cache_time(cache_time),
  interpolating (interpolating),
  max_extrapolation_distance(max_extrapolation_distance)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back( new RefFrame(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

TransformReference::~TransformReference()
{
  /* deallocate all frames */
  frame_mutex_.lock();
  for (std::vector<RefFrame*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  {
    delete (*it);
  }
  frame_mutex_.unlock();

};

void TransformReference::addFrame(unsigned int frame_id, unsigned int parent_id) {


  getFrame(frame_id)->setParent(parent_id);
}


void TransformReference::setWithEulers(const std::string & frameID, const std::string & parentID, double a,double b,double c,double d,double e,double f, ULLtime time)
{
  unsigned int frame_id_num = lookup(frameID);
  unsigned int parent_id_num = lookup(parentID);
  addFrame(frame_id_num, parent_id_num);

  getFrame(frame_id_num)->addFromEuler(a,b,c,d,e,f,time);
}

void TransformReference::setWithDH(const std::string & frameID, const std::string & parentID, double a,double b,double c,double d, ULLtime time)
{
  unsigned int frame_id_num = lookup(frameID);
  unsigned int parent_id_num = lookup(parentID);
  addFrame(frame_id_num, parent_id_num);

  getFrame(frame_id_num)->addFromDH(a,b,c,d,time);
}


void TransformReference::setWithMatrix(const std::string & frameID, const std::string & parentID, const NEWMAT::Matrix & matrix_in, ULLtime time)
{
  unsigned int frame_id_num = lookup(frameID);
  unsigned int parent_id_num = lookup(parentID);
  addFrame(frame_id_num, parent_id_num);

  getFrame(frame_id_num)->addFromMatrix(matrix_in,time);
}


void TransformReference::setWithQuaternion(const std::string & frameID, const std::string & parentID, double xt, double yt, double zt, double xr, double yr, double zr, double w, ULLtime time)
{
  unsigned int frame_id_num = lookup(frameID);
  unsigned int parent_id_num = lookup(parentID);
  addFrame(frame_id_num, parent_id_num);

  getFrame(frame_id_num)->addFromQuaternion(xt, yt, zt, xr, yr, zr, w,time);
}


void TransformReference::clear()
{
  frame_mutex_.lock();
  for (std::vector< RefFrame*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  {
    (*it)->clearList();
  }
  frame_mutex_.unlock();
}



NEWMAT::Matrix TransformReference::getMatrix(const std::string & target_frame, const std::string & source_frame, ULLtime time)
{
  unsigned int target_frame_num = lookup(target_frame);
  unsigned int source_frame_num = lookup(source_frame);
  NEWMAT::Matrix myMat(4,4);
  TransformLists lists = lookUpList(target_frame_num, source_frame_num);
  myMat = computeTransformFromList(lists,time);
  return myMat;
}




TFPoint TransformReference::transformPoint(const std::string & target_frame, const TFPoint & point_in)
{
  //Create a vector
  NEWMAT::Matrix pointMat(4,1);
  pointMat << point_in.x << point_in.y << point_in.z << 1;

  NEWMAT::Matrix myMat = getMatrix(target_frame, point_in.frame, point_in.time);


  pointMat = myMat * pointMat;
  TFPoint retPoint;
  retPoint.x = pointMat(1,1);
  retPoint.y = pointMat(2,1);
  retPoint.z = pointMat(3,1);
  retPoint.frame = target_frame;
  retPoint.time = point_in.time;
  return retPoint;
}
TFPoint2D TransformReference::transformPoint2D(const std::string & target_frame, const TFPoint2D & point_in)
{
  //Create a vector
  NEWMAT::Matrix pointMat(4,1);
  pointMat << point_in.x << point_in.y << 0 << 1;  //no Z element

  NEWMAT::Matrix myMat = getMatrix(target_frame, point_in.frame, point_in.time);


  pointMat = myMat * pointMat;
  TFPoint2D retPoint;
  retPoint.x = pointMat(1,1);
  retPoint.y = pointMat(2,1);
  retPoint.frame = target_frame;
  retPoint.time = point_in.time;
  return retPoint;
}

TFVector TransformReference::transformVector(const std::string & target_frame, const TFVector & vector_in)
{
  //Create a vector
  NEWMAT::Matrix vectorMat(4,1);
  vectorMat << vector_in.x << vector_in.y << vector_in.z << 0; // 0 vs 1 only difference between point and vector //fixme make this less copy and paste

  NEWMAT::Matrix myMat = getMatrix(target_frame, vector_in.frame, vector_in.time);

  vectorMat = myMat * vectorMat;
  TFVector retVector;
  retVector.x = vectorMat(1,1);
  retVector.y = vectorMat(2,1);
  retVector.z = vectorMat(3,1);
  retVector.frame = target_frame;
  retVector.time = vector_in.time;
  return retVector;
}

TFVector2D TransformReference::transformVector2D(const std::string & target_frame, const TFVector2D & vector_in)
{
  //Create a vector
  NEWMAT::Matrix vectorMat(4,1);
  vectorMat << vector_in.x << vector_in.y << 0 << 0; // 0 vs 1 only difference between point and vector //fixme make this less copy and paste
  // no Z

  NEWMAT::Matrix myMat = getMatrix(target_frame, vector_in.frame, vector_in.time);


  vectorMat = myMat * vectorMat;
  TFVector2D retVector;
  retVector.x = vectorMat(1,1);
  retVector.y = vectorMat(2,1);
  retVector.frame = target_frame;
  retVector.time = vector_in.time;
  return retVector;
}

TFEulerYPR TransformReference::transformEulerYPR(const std::string & target_frame, const TFEulerYPR & euler_in)
{

  NEWMAT::Matrix local = Pose3D::matrixFromEuler(0,0,0,euler_in.yaw, euler_in.pitch, euler_in.roll);
  NEWMAT::Matrix Transform = getMatrix(target_frame, euler_in.frame, euler_in.time);

  NEWMAT::Matrix output = local.i() * Transform;

  Euler eulers = Pose3D::eulerFromMatrix(output,1);

  TFEulerYPR retEuler;
  retEuler.yaw = eulers.yaw;
  retEuler.pitch = eulers.pitch;
  retEuler.roll = eulers.roll;
  return retEuler;
}

TFYaw  TransformReference::transformYaw(const std::string & target_frame, const TFYaw & euler_in)
{
  TFVector2D vector_in;
  vector_in.x = cos(euler_in.yaw);
  vector_in.y = sin(euler_in.yaw);
  vector_in.frame = euler_in.frame;
  vector_in.time = euler_in.time;
  TFVector2D vector_out = transformVector2D(target_frame, vector_in);

  TFYaw retYaw;
  retYaw.yaw = atan2(vector_out.y, vector_out.x);
  retYaw.frame = target_frame;
  retYaw.time = euler_in.time;
  return retYaw;
}

TFPose TransformReference::transformPose(const std::string & target_frame, const TFPose & pose_in)
{
  TFPoint point_in;
  point_in.x = pose_in.x;
  point_in.y = pose_in.y;
  point_in.z = pose_in.z;
  point_in.frame = pose_in.frame;
  point_in.time = pose_in.time;
  TFPoint point_out = transformPoint(target_frame, point_in);

  TFPose pose_out;
  pose_out.x = point_out.x;
  pose_out.y = point_out.y;
  pose_out.z = point_out.z;

  TFEulerYPR eulers_in;
  eulers_in.yaw = pose_in.yaw;
  eulers_in.pitch = pose_in.pitch;
  eulers_in.roll = pose_in.roll;
  eulers_in.frame = pose_in.frame;
  eulers_in.time = pose_in.time;

  TFEulerYPR eulers_out = transformEulerYPR(target_frame, eulers_in);

  pose_out.yaw = eulers_out.yaw;
  pose_out.pitch = eulers_out.pitch;
  pose_out.roll = eulers_out.roll;

  pose_out.time = pose_in.time;
  pose_out.frame = target_frame;
  return pose_out;
}

TFPose2D TransformReference::transformPose2D(const std::string & target_frame, const TFPose2D & pose_in)
{
  TFPoint2D point_in;
  point_in.x = pose_in.x;
  point_in.y = pose_in.y;
  point_in.frame = pose_in.frame;
  point_in.time = pose_in.time;
  TFPoint2D point_out = transformPoint2D(target_frame, point_in);

  TFYaw yaw;
  yaw.yaw = pose_in.yaw;
  yaw.time = pose_in.time;
  yaw.frame = pose_in.frame;
  TFYaw yaw_out = transformYaw(target_frame, yaw);

  TFPose2D pose_out;
  pose_out.x = point_out.x;
  pose_out.y = point_out.y;
  pose_out.yaw = yaw_out.yaw;
  pose_out.time = pose_in.time;
  pose_out.frame = target_frame;
  return pose_out;
}



TransformReference::TransformLists TransformReference::lookUpList(unsigned int target_frame,unsigned int source_frame)
{
  /*  timeval tempt;
  gettimeofday(&tempt,NULL);
  std::cerr << "Looking up list at " <<tempt.tv_sec * 1000000ULL + tempt.tv_usec << std::endl;
  */

  TransformLists mTfLs;
  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  if (getFrame(frame) == NULL) //Test if source frame exists this will throw a lookup error if it does not (inside the loop it will be caught)
    throw LookupException("fixme");
  while (true)
    {
      unsigned int parent;

      RefFrame* pointer = getFrame(frame);
      if (pointer == NULL) break;
      parent = pointer->getParent();

      /*      try
      {
        parent = getFrame(frame)->getParent();
      }
      catch (TransformReference::LookupException & ex)
      {
        //This frame doesn't exist must be at top of list
        break;
        }*/
      mTfLs.inverseTransforms.push_back(frame);
      frame = parent;


      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH){
        std::stringstream ss;
        ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
          << viewFrames() << std::endl;
        throw(MaxDepthException(ss.str()));
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

      unsigned int parent;
      RefFrame* pointer = getFrame(frame);
      if (pointer == NULL) break;
      parent = pointer->getParent();

      /*      try
      {
        parent = getFrame(frame)->getParent();
      }
      catch (TransformReference::LookupException & ex)
      {
        //This frame doesn't exist don't add to stack
        break;
        }*/
      mTfLs.forwardTransforms.push_back(frame);
      frame = parent;

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH){
        std::stringstream ss;
        ss<<"Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph" << std::endl
          << viewFrames() << std::endl;
        throw(MaxDepthException(ss.str()));
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
      ss<< "No Common ParentD" << std::endl << viewFrames() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    if (mTfLs.forwardTransforms.back() != source_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentC" << std::endl << viewFrames() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
  }

  if (mTfLs.forwardTransforms.size() == 0)
  {
    if (mTfLs.inverseTransforms.size() == 0)
    {
      std::stringstream ss;
      ss<< "No Common ParentB" << std::endl << viewFrames() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
    if (mTfLs.inverseTransforms.back() != target_frame)
    {
      std::stringstream ss;
      ss<< "No Common ParentA" << std::endl << viewFrames() << std::endl;
      throw(ConnectivityException(ss.str()));
    }
  }

  /* Make sure the end of the search shares a parent. */
  if (mTfLs.inverseTransforms.back() != mTfLs.forwardTransforms.back())
  {
    std::stringstream ss;
    ss<< "No Common Parent, at top of search" << std::endl << viewFrames() << std::endl;
    throw(ConnectivityException(ss.str()));
  }
  /* Make sure that we don't have a no parent at the top */
  if (mTfLs.inverseTransforms.back() == 0 ||  mTfLs.forwardTransforms.back() == 0)
    throw(ConnectivityException("NO_PARENT at top of tree"));
  /*
  gettimeofday(&tempt2,NULL);
  std::cerr << "Base Cases done" <<tempt.tv_sec * 1000000LL + tempt.tv_usec- tempt2.tv_sec * 1000000LL - tempt2.tv_usec << std::endl;
  */

  while (mTfLs.inverseTransforms.back() == mTfLs.forwardTransforms.back())
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

NEWMAT::Matrix TransformReference::computeTransformFromList(const TransformLists & lists, ULLtime time)
{
  NEWMAT::Matrix retMat(4,4);
  retMat << 1 << 0 << 0 << 0
	 << 0 << 1 << 0 << 0
	 << 0 << 0 << 1 << 0
	 << 0 << 0 << 0 << 1;

  ///@todo change these to iterators
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      try {
        retMat *= getFrame(lists.inverseTransforms[lists.inverseTransforms.size() -1 - i])->getMatrix(time); //Reverse to get left multiply
      }
      catch (libTF::Pose3DCache::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.inverseTransforms[lists.inverseTransforms.size() -1 - i] << " is out of date. " << ex.what();
        throw ExtrapolateException(ss.str());
      }
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      try {
        retMat = getFrame(lists.forwardTransforms[lists.forwardTransforms.size() -1 - i])->getInverseMatrix(time) * retMat;  //Do this list backwards(from backwards) for it was generated traveling the wrong way
      }
      catch (libTF::Pose3DCache::ExtrapolationException &ex)
      {
        std::stringstream ss;
        ss << "Frame "<< lists.forwardTransforms[lists.forwardTransforms.size() -1 - i] << " is out of date. " << ex.what();
        throw ExtrapolateException(ss.str());
      }
    }

  return retMat;
}


std::string TransformReference::viewChain(const std::string & target_frame, const std::string & source_frame)
{
  stringstream mstream;
  TransformLists lists = lookUpList(lookup(target_frame), lookup(source_frame));

  mstream << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      mstream << lists.inverseTransforms[i]<<", ";
    }
  mstream << std::endl;

  mstream << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++)
    {
      mstream << lists.forwardTransforms[i]<<", ";
    }
  mstream << std::endl;
  return mstream.str();
}

std::string TransformReference::viewFrames()
{
  stringstream mstream;
  frame_mutex_.lock();

  //  for (std::vector< RefFrame*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    mstream << "Frame "<< counter << " exists with parent " << frames_[counter]->getParent() << "." <<std::endl;
  }
  frame_mutex_.unlock();
  return mstream.str();
}


bool TransformReference::RefFrame::setParent(unsigned int parent_id)
{
  if (parent_ != parent_id)
  {
    parent_ = parent_id;
    clearList();
    return false;
  }
  return true;
};


TransformReference::RefFrame* TransformReference::getFrame(unsigned int frame_id)
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else
    return frames_[frame_id];

  /*  frame_mutex_.lock();
  std::map<unsigned int, RefFrame*>::const_iterator it = frames_.find(frame_id);
  bool found = it != frames_.end();
  RefFrame *frame = found ? it->second : NULL;
  frame_mutex_.unlock();

  if (!found){
    return NULL; // @todo check where HOBBLED THROW may effect
    std::stringstream ss; ss << "getFrame: Frame " << frame_id  << " does not exist."
                             << " Frames Present are: " <<std::endl << viewFrames() <<std::endl;
    throw LookupException(ss.str());
  }
  return frame;
  */
};
