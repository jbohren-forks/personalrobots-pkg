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

#include "libTF.hh"

RefFrame::RefFrame() :
  parent(0),
  myQuat(0,0,0,0,0,0,1,111110000)
{
  return;
}

/* Quaternion 3D version */
void RefFrame::setParamsQuaternion3D(double a,double b,double c,double d,double e,double f, double g, unsigned long long time)
{
  myQuat.Set(a,b,c,d,e,f,g,time);
};

/* Six DOF version */
void RefFrame::setParamsEulers(double a,double b,double c,double d,double e,double f, unsigned long long time)
{
  myQuat.fromEuler(a,b,c,d,e,f,time) ;
}

/* DH Params version */
void RefFrame::setParamsDH(double a,double b,double c,double d, unsigned long long time)
{
  myQuat.fromDH(a,b,c,d,time);
}


NEWMAT::Matrix RefFrame::getMatrix(unsigned long long time)
{
  return myQuat.asMatrix(time);
}

NEWMAT::Matrix RefFrame::getInverseMatrix(unsigned long long time)
{
  return myQuat.asMatrix(time).i();
};


TransformReference::TransformReference()
{
  /* initialize pointers to NULL */
  for (unsigned int i = 0; i < MAX_NUM_FRAMES; i++)
    {
      frames[i] = NULL;
    }
  return;
}


void TransformReference::set(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d,double e,double f, unsigned long long time)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();
  
  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->setParamsEulers(a,b,c,d,e,f,time);
}


NEWMAT::Matrix TransformReference::get(unsigned int target_frame, unsigned int source_frame, unsigned long long time)
{
  NEWMAT::Matrix myMat(4,4);
  TransformLists lists = lookUpList(target_frame, source_frame);
  myMat = computeTransformFromList(lists,time);
  //  std::cout << myMat;
  return myMat;
}


TransformReference::TransformLists TransformReference::lookUpList(unsigned int target_frame, unsigned int source_frame)
{
  TransformLists mTfLs;

  //  std::vector<unsigned int> tList;
  //  std::vector<unsigned int> sList;

  //  std::vector<unsigned int> retVec;

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
      //   std::cout <<"Frame: " << frame <<std::endl;
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
  //  std::cout << "Back = " << mTfLs.inverseTransforms.back()<<" " << mTfLs.forwardTransforms.back();
  if (mTfLs.inverseTransforms.back() == NO_PARENT ||  mTfLs.forwardTransforms.back() == NO_PARENT)
    throw(NoFrameConnectivity);

  while (mTfLs.inverseTransforms.back() == mTfLs.forwardTransforms.back())
    {
      //      std::cout << "removing " << mTfLs.inverseTransforms.back() << std::endl;
      mTfLs.inverseTransforms.pop_back();
      mTfLs.forwardTransforms.pop_back();
    }
  
  return mTfLs;

}

NEWMAT::Matrix TransformReference::computeTransformFromList(TransformLists lists, unsigned long long time)
{
  NEWMAT::Matrix retMat(4,4);
  retMat << 1 << 0 << 0 << 0
	 << 0 << 1 << 0 << 0
	 << 0 << 0 << 1 << 0
	 << 0 << 0 << 0 << 1;
  
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      retMat *= getFrame(lists.inverseTransforms[i])->getInverseMatrix(time);
      //      std::cout <<"Multiplying by " << std::endl << frames[lists.inverseTransforms[i]].getInverseMatrix() << std::endl; 
      //std::cout <<"Result "<<std::endl << retMat << std::endl;
   }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      retMat *= getFrame(lists.forwardTransforms[lists.forwardTransforms.size() -1 - i])->getMatrix(time); //Do this list backwards for it was generated traveling the wrong way
      //      std::cout <<"Multiplying by "<<std::endl << frames[lists.forwardTransforms[i]].getMatrix() << std::endl;
      //std::cout <<"Result "<<std::endl << retMat << std::endl;
    }

  return retMat;
}


void TransformReference::view(unsigned int target_frame, unsigned int source_frame)
{
  TransformLists lists = lookUpList(target_frame, source_frame);

  std::cout << "Inverse Transforms:" <<std::endl;
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      std::cout << lists.inverseTransforms[i]<<", ";
      //      retMat *= getFrame(lists.inverseTransforms[i])->getInverseMatrix();
    }
  std::cout << std::endl;

  std::cout << "Forward Transforms: "<<std::endl ;
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      std::cout << lists.forwardTransforms[i]<<", ";
      //      retMat *= getFrame(lists.inverseTransforms[lists.forwardTransforms.size() -1 - i])->getMatrix(); //Do this list backwards for it was generated traveling the wrong way
    }
  std::cout << std::endl;
  
}
