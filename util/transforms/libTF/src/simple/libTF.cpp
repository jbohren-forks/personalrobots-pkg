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

using namespace libTF;

RefFrame::RefFrame() :
  Quaternion3D(),
  parent(TransformReference::NO_PARENT)
{
  return;
}


TransformReference::TransformReference(ULLtime cache_time):
  cache_time(cache_time)
{
  /* initialize pointers to NULL */
  for (unsigned int i = 0; i < MAX_NUM_FRAMES; i++)
    {
      frames[i] = NULL;
    }
  return;
}


void TransformReference::setWithEulers(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d,double e,double f, ULLtime time)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();
  
  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->fromEuler(a,b,c,d,e,f,time);
}

void TransformReference::setWithDH(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d, ULLtime time)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();
  
  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->fromDH(a,b,c,d,time);
}


void TransformReference::setWithMatrix(unsigned int frameID, unsigned int parentID, const NEWMAT::Matrix & matrix_in, ULLtime time)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  //TODO check and throw exception if matrix wrong size
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();

  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->fromMatrix(matrix_in,time);
}


void TransformReference::setWithQuaternion(unsigned int frameID, unsigned int parentID, double xt, double yt, double zt, double xr, double yr, double zr, double w, ULLtime time)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();
  
  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->fromQuaternion(xt, yt, zt, xr, yr, zr, w,time);
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


TransformReference::TransformLists TransformReference::lookUpList(unsigned int target_frame, unsigned int source_frame)
{
  TransformLists mTfLs;

  unsigned int frame = target_frame;
  unsigned int counter = 0;  //A counter to keep track of how deep we've descended
  while (true)
    {
      if (frame == NO_PARENT)
	break;
      mTfLs.inverseTransforms.push_back(frame);
      if (frame == ROOT_FRAME) //Descend until we reach the root node or don't have a parent
	break;

      //Check that we arn't going somewhere illegal 
      if (getFrame(frame)->getParent() > MAX_NUM_FRAMES) throw InvalidFrame;

      // Descent to parent frame
      frame = getFrame(frame)->getParent();

      /* Check if we've gone too deep.  A loop in the tree would cause this */
      if (counter++ > MAX_GRAPH_DEPTH)
	throw(MaxSearchDepth);
    }
  
  frame = source_frame;
  counter = 0;
  while (true)
    {
      if (frame == NO_PARENT)
	break;
      mTfLs.forwardTransforms.push_back(frame);
      if (frame == ROOT_FRAME) //Descend until we reach the root node or don't have a parent
	break;

      //Check that we aren't going somewhere illegal
      if (getFrame(frame)->getParent() > MAX_NUM_FRAMES) throw InvalidFrame;

      //Descent to parent frame
      frame = getFrame(frame)->getParent();

      /* Check if we've gone too deep.  A loop in the tree would cause this*/
      if (counter++ > MAX_GRAPH_DEPTH)
	throw(MaxSearchDepth);
    }
  
  /* Make sure the end of the search shares a parent. */
  if (mTfLs.inverseTransforms.back() != mTfLs.forwardTransforms.back())
    throw(NoFrameConnectivity);

  /* Make sure that we don't have a no parent at the top */
  if (mTfLs.inverseTransforms.back() == NO_PARENT ||  mTfLs.forwardTransforms.back() == NO_PARENT)
    throw(NoFrameConnectivity);

  bool imdone = false;
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
      retMat *= getFrame(lists.inverseTransforms[i])->getMatrix(time);
   }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      retMat *= getFrame(lists.forwardTransforms[lists.forwardTransforms.size() -1 - i])->getInverseMatrix(time); //Do this list backwards for it was generated traveling the wrong way
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

TransformReference::ULLtime TransformReference::gettime()
{
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  return temp_time_struct.tv_sec * 1000000000ULL + (unsigned long long)temp_time_struct.tv_usec * 1000ULL;
}

