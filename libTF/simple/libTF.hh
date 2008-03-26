#ifndef LIBTF_HH
#define LIBTF_HH
#include <iostream>
#include <iomanip>
#include <newmat/newmat.h>
#include <newmat/newmatio.h>
#include <math.h>
#include <vector>

class RefFrame
{
public:
  //  static enum FrameType {SIXDOF, DH //how to tell which mode it's in  //todo add this process i'm going to start with 6dof only
  RefFrame();
  void setParams(double, double, double, double, double, double);
  inline unsigned int getParent(){return parent;};
  inline void setParent(unsigned int parentID){parent = parentID;};
  NEWMAT::Matrix getMatrix();
  NEWMAT::Matrix getInverseMatrix();
private:
  double params[6];
  static  bool fill_transformation_matrix(NEWMAT::Matrix& matrix_pointer, double ax,
					  double ay, double az, double yaw,
					  double pitch, double roll);
  unsigned int parent;
  bool active;

};

class TransformReference
{
 public:
  void set(unsigned int framid, unsigned int parentid, double,double,double,double,double,double);
  NEWMAT::Matrix get(unsigned int target_frame, unsigned int source_frame);

  TransformReference();
  void view(unsigned int target_frame, unsigned int source_frame);
 private:
  double rD[6];
  NEWMAT::Matrix mMat;
  RefFrame frames[100];

  typedef struct 
  {
    std::vector<unsigned int> inverseTransforms;
    std::vector<unsigned int> forwardTransforms;
  } TransformLists;

  TransformLists  lookUpList(unsigned int target_frame, unsigned int source_frame);
  NEWMAT::Matrix computeTransformFromList(TransformLists list);

};
#endif //LIBTF_HH
