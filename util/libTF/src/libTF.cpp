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

using namespace libTF;

TransformReference::RefFrame::RefFrame(bool interpolating, 
                                       unsigned long long max_cache_time,
                                       unsigned long long max_extrapolation_distance) :
  Pose3DCache(interpolating, max_cache_time, max_extrapolation_distance),
  parent(TransformReference::NO_PARENT)
{
  return;
}


TransformReference::TransformReference(bool interpolating, 
                                       ULLtime cache_time,
                                       unsigned long long max_extrapolation_distance):
  cache_time(cache_time),
  interpolating (interpolating),
  max_extrapolation_distance(max_extrapolation_distance)
{
  
  frames = new RefFrame*[MAX_NUM_FRAMES];
  assert(frames);
  
  /* initialize pointers to NULL */
  for (unsigned int i = 0; i < MAX_NUM_FRAMES; i++)
    {
      frames[i] = NULL;
    }
  return;
}

TransformReference::~TransformReference()
{
  /* initialize pointers to NULL */
  for (unsigned int i = 0; i < MAX_NUM_FRAMES; i++)
    {
      if (frames[i] != NULL)
	delete frames[i];
    }
  
  delete[] frames;
};

void TransformReference::addFrame(unsigned int frameID, unsigned int parentID) {
  if (frameID >= MAX_NUM_FRAMES || parentID >= MAX_NUM_FRAMES || frameID == NO_PARENT)
  {
    std::stringstream ss;
    ss << "frameID("<<frameID<<") >= MAX_NUM_FRAMES || parentID("<<parentID<<") >= MAX_NUM_FRAMES || frameID("<<frameID<<") == NO_PARENT";
    throw LookupException(ss.str());
  }

  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame(interpolating, cache_time, max_extrapolation_distance);

  if (frames[parentID] == NULL)
    frames[parentID] = new RefFrame(interpolating, cache_time, max_extrapolation_distance);

  getFrame(frameID)->setParent(parentID);
}


void TransformReference::setWithEulers(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d,double e,double f, ULLtime time)
{
  addFrame(frameID, parentID);

  getFrame(frameID)->addFromEuler(a,b,c,d,e,f,time);
}

void TransformReference::setWithDH(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d, ULLtime time)
{
  addFrame(frameID, parentID);

  getFrame(frameID)->addFromDH(a,b,c,d,time);
}


void TransformReference::setWithMatrix(unsigned int frameID, unsigned int parentID, const NEWMAT::Matrix & matrix_in, ULLtime time)
{
  addFrame(frameID, parentID);

  getFrame(frameID)->addFromMatrix(matrix_in,time);
}


void TransformReference::setWithQuaternion(unsigned int frameID, unsigned int parentID, double xt, double yt, double zt, double xr, double yr, double zr, double w, ULLtime time)
{
  addFrame(frameID, parentID);

  getFrame(frameID)->addFromQuaternion(xt, yt, zt, xr, yr, zr, w,time);
}




NEWMAT::Matrix TransformReference::getMatrix(unsigned int target_frame, unsigned int source_frame, ULLtime time)
{
  NEWMAT::Matrix myMat(4,4);
  TransformLists lists = lookUpList(target_frame, source_frame);
  myMat = computeTransformFromList(lists,time);
  return myMat;
}




TFPoint TransformReference::transformPoint(unsigned int target_frame, const TFPoint & point_in)
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
TFPoint2D TransformReference::transformPoint2D(unsigned int target_frame, const TFPoint2D & point_in)
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

TFVector TransformReference::transformVector(unsigned int target_frame, const TFVector & vector_in)
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

TFVector2D TransformReference::transformVector2D(unsigned int target_frame, const TFVector2D & vector_in)
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

TFEulerYPR TransformReference::transformEulerYPR(unsigned int target_frame, const TFEulerYPR & euler_in)
{

  NEWMAT::Matrix local = Pose3D::matrixFromEuler(0,0,0,euler_in.yaw, euler_in.pitch, euler_in.roll);
  NEWMAT::Matrix Transform = getMatrix(target_frame, euler_in.frame, euler_in.time);
  
  NEWMAT::Matrix output = local.i() * Transform;

  Pose3D::Euler eulers = Pose3D::eulerFromMatrix(output,1); 

  TFEulerYPR retEuler;
  retEuler.yaw = eulers.yaw;
  retEuler.pitch = eulers.pitch;
  retEuler.roll = eulers.roll;
  return retEuler;
}

TFYaw  TransformReference::transformYaw(unsigned int target_frame, const TFYaw & euler_in)
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

TFPose TransformReference::transformPose(unsigned int target_frame, const TFPose & pose_in)
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

TFPose2D TransformReference::transformPose2D(unsigned int target_frame, const TFPose2D & pose_in)
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



TransformReference::TransformLists TransformReference::lookUpList(unsigned int target_frame, unsigned int source_frame)
{
  TransformLists mTfLs;
  if (source_frame == NO_PARENT) throw LookupException("cannnot lookup source frame id = NO_PARENT (0)"); 
  if (target_frame == NO_PARENT) throw LookupException("cannnot lookup target frame id = NO_PARENT (0)"); 

  unsigned int frame = source_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  while (true)
    {
      if (frame == NO_PARENT)
	break;
      mTfLs.inverseTransforms.push_back(frame);

      //Check that we arn't going somewhere illegal 
      unsigned int parentid = getFrame(frame)->getParent();
      if (getFrame(frame)->getParent() >=MAX_NUM_FRAMES)
      {
        std::stringstream ss;
        ss <<"parentid("<<parentid<<") greater than MAX_NUM_FRAMES("<<MAX_NUM_FRAMES<<")";
        throw LookupException(ss.str());
      }
      
      // Descent to parent frame
      frame = getFrame(frame)->getParent();

      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH)
	throw(MaxDepthException("Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph"));
    }
  
  frame = target_frame;
  counter = 0;
  while (true)
    {
      if (frame == NO_PARENT)
	break;
      mTfLs.forwardTransforms.push_back(frame);

      //Check that we aren't going somewhere illegal
      unsigned int parentid = getFrame(frame)->getParent();
      if ( parentid >= MAX_NUM_FRAMES)
      {
        std::stringstream ss;
        ss<< "parentid("<<parentid<<") of frame ("<<frame<<") greater than MAX_NUM_FRAMES("<<MAX_NUM_FRAMES<<")";
        throw LookupException(ss.str());
      }

      //Descent to parent frame
      frame = getFrame(frame)->getParent();

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH)
	throw(MaxDepthException("Recursed too deep into graph ( > MAX_GRAPH_DEPTH) there is probably a loop in the graph"));
    }
  
  /* Make sure the end of the search shares a parent. */
  if (mTfLs.inverseTransforms.back() != mTfLs.forwardTransforms.back())
    throw(ConnectivityException("No Common Parent, at top of search"));

  /* Make sure that we don't have a no parent at the top */
  if (mTfLs.inverseTransforms.back() == NO_PARENT ||  mTfLs.forwardTransforms.back() == NO_PARENT)
    throw(ConnectivityException("No Common Parent"));

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
  
  return mTfLs;

}

NEWMAT::Matrix TransformReference::computeTransformFromList(const TransformLists & lists, ULLtime time)
{
  NEWMAT::Matrix retMat(4,4);
  retMat << 1 << 0 << 0 << 0
	 << 0 << 1 << 0 << 0
	 << 0 << 0 << 1 << 0
	 << 0 << 0 << 0 << 1;
  

  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      retMat *= getFrame(lists.inverseTransforms[lists.inverseTransforms.size() -1 - i])->getMatrix(time); //Reverse to get left multiply
    }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      retMat *= getFrame(lists.forwardTransforms[i])->getInverseMatrix(time); //Do this list backwards(from backwards) for it was generated traveling the wrong way
    }

  return retMat;
}


std::string TransformReference::viewChain(unsigned int target_frame, unsigned int source_frame)
{
  stringstream mstream;
  TransformLists lists = lookUpList(target_frame, source_frame);

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
  for (unsigned int frameid = 1; frameid < MAX_NUM_FRAMES; frameid++)
  {
    if (frames[frameid] != NULL)
    {
      mstream << "Frame "<< frameid << " exists with parent " << frames[frameid]->getParent() << "." <<std::endl;    
    }
  }
  return mstream.str();
}


bool TransformReference::RefFrame::setParent(unsigned int parentID)
{
  if (parent != parentID)
    {
      parent = parentID; 
      clearList(); 
      return false;
    } 
  return true;
};
