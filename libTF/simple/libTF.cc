#include "libTF.hh"

RefFrame::RefFrame() :
  parent(0)
{
  return;
}

/* Six DOF version */
void RefFrame::setParamsXYZYPR(double a,double b,double c,double d,double e,double f)
{
  paramType = SIXDOF;
  params[0]=a;
  params[1]=b;
  params[2]=c;
  params[3]=d;
  params[4]=e;
  params[5]=f;

}

/* DH Params version */
void RefFrame::setParamsDH(double a,double b,double c,double d)
{
  paramType = DH;
  params[0]=a;
  params[1]=b;
  params[2]=c;
  params[3]=d;
}


NEWMAT::Matrix RefFrame::getMatrix()
{
  NEWMAT::Matrix mMat(4,4);
  switch ( paramType)
    {
    case SIXDOF:
      fill_transformation_matrix(mMat,params[0],params[1],params[2],params[3],params[4],params[5]);
      break;
    case DH:
      fill_transformation_matrix_from_dh(mMat,params[0],params[1],params[2],params[3]);
    }
  return mMat;
}

NEWMAT::Matrix RefFrame::getInverseMatrix()
{
  NEWMAT::Matrix mMat(4,4);
  switch(paramType)
    {
    case SIXDOF:
      fill_transformation_matrix(mMat,params[0],params[1],params[2],params[3],params[4],params[5]);
      //todo create a fill_inverse_transform_matrix call to be more efficient
      return mMat.i();
      break;
    case DH:
      fill_transformation_matrix_from_dh(mMat,params[0],params[1],params[2],params[3]);
      //todo create a fill_inverse_transform_matrix call to be more efficient
      return mMat.i();
      break;
    }
}


TransformReference::TransformReference()
{
  /* initialize pointers to NULL */
  for (unsigned int i = 0; i < MAX_NUM_FRAMES; i++)
    {
      frames[i] = NULL;
    }
  return;
}


void TransformReference::set(unsigned int frameID, unsigned int parentID, double a,double b,double c,double d,double e,double f)
{
  if (frameID > MAX_NUM_FRAMES || parentID > MAX_NUM_FRAMES || frameID == NO_PARENT || frameID == ROOT_FRAME)
    throw InvalidFrame;
  
  if (frames[frameID] == NULL)
    frames[frameID] = new RefFrame();
  
  getFrame(frameID)->setParent(parentID);
  getFrame(frameID)->setParamsXYZYPR(a,b,c,d,e,f);
}


NEWMAT::Matrix TransformReference::get(unsigned int target_frame, unsigned int source_frame)
{
  NEWMAT::Matrix myMat(4,4);
  TransformLists lists = lookUpList(target_frame, source_frame);
  myMat = computeTransformFromList(lists);
  //  std::cout << myMat;
  return myMat;
}




bool RefFrame::fill_transformation_matrix(NEWMAT::Matrix & matrix, double ax,
                                                   double ay, double az, double yaw,
                                                   double pitch, double roll)
{
  double ca = cos(yaw);
  double sa = sin(yaw);
  double cb = cos(pitch);
  double sb = sin(pitch);
  double cg = cos(roll);
  double sg = sin(roll);
  double sbsg = sb*sg;
  double sbcg = sb*cg;


  double* matrix_pointer = matrix.Store();
  if (matrix.Storage() != 16)
    return false;

  matrix_pointer[0] =  ca*cb;
  matrix_pointer[1] = (ca*sbsg)-(sa*cg);
  matrix_pointer[2] = (ca*sbcg)+(sa*sg);
  matrix_pointer[3] = ax;
  matrix_pointer[4] = sa*cb;
  matrix_pointer[5] = (sa*sbsg)+(ca*cg);
  matrix_pointer[6] = (sa*sbcg)-(ca*sg);
  matrix_pointer[7] = ay;
  matrix_pointer[8] = -sb;
  matrix_pointer[9] = cb*sg;
  matrix_pointer[10] = cb*cg;
  matrix_pointer[11] = az;
  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return true;
};

// Math from http://en.wikipedia.org/wiki/Robotics_conventions
bool RefFrame::fill_transformation_matrix_from_dh(NEWMAT::Matrix& matrix, double theta,
						 double length, double distance, double alpha)
{
  double ca = cos(alpha);
  double sa = sin(alpha);
  double ct = cos(theta);
  double st = sin(theta);

  double* matrix_pointer = matrix.Store();
  if (matrix.Storage() != 16)
    return false;

  matrix_pointer[0] =  ct;
  matrix_pointer[1] = -st*ca;
  matrix_pointer[2] = st*sa;
  matrix_pointer[3] = distance * ct;
  matrix_pointer[4] = st;
  matrix_pointer[5] = ct*ca;
  matrix_pointer[6] = -ct*sa;
  matrix_pointer[7] = distance*st;
  matrix_pointer[8] = 0;
  matrix_pointer[9] = sa;
  matrix_pointer[10] = ca;
  matrix_pointer[11] = length;
  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return true;
};



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

NEWMAT::Matrix TransformReference::computeTransformFromList(TransformLists lists)
{
  NEWMAT::Matrix retMat(4,4);
  retMat << 1 << 0 << 0 << 0
	 << 0 << 1 << 0 << 0
	 << 0 << 0 << 1 << 0
	 << 0 << 0 << 0 << 1;
  
  for (unsigned int i = 0; i < lists.inverseTransforms.size(); i++)
    {
      retMat *= getFrame(lists.inverseTransforms[i])->getInverseMatrix();
      //      std::cout <<"Multiplying by " << std::endl << frames[lists.inverseTransforms[i]].getInverseMatrix() << std::endl; 
      //std::cout <<"Result "<<std::endl << retMat << std::endl;
   }
  for (unsigned int i = 0; i < lists.forwardTransforms.size(); i++) 
    {
      retMat *= getFrame(lists.forwardTransforms[lists.forwardTransforms.size() -1 - i])->getMatrix(); //Do this list backwards for it was generated traveling the wrong way
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
