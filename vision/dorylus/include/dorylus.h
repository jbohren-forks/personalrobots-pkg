#ifndef DORYLUS_H
#define DORYLUS_H

#include <iomanip>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cassert>
#include <stdlib.h>
#include <sstream>

#define Matrix NEWMAT::Matrix
//using namespace NEWMAT;

typedef struct 
{
  string descriptor;
  Matrix center;
  float theta;
  Matrix vals;
} weak_classifier;

typedef struct 
{
  int label;
  map<string, Matrix> features;
} object;

typedef float Real;

inline float euc(Matrix a, Matrix b)
{
  assert(a.Ncols() == 1 && b.Ncols() == 1);
  return (a-b).NormFrobenius();
}

class DorylusDataset {
 public:
  vector<object> objs_;
  Matrix ymc_;
  unsigned int nClasses_;

 DorylusDataset(unsigned int nClasses) : nClasses_(nClasses)
  
  {
  }

  std::string status()
  {
    ostringstream oss (ostringstream::out);

    oss << "DorylusDataset status: \n";
    oss << "  nClasses: " << nClasses_ << "\n";
    return oss.str();
  }

  void setObjs(vector<object> *objs) {
    objs_ = *objs;
    ymc_ = Matrix(nClasses_, objs_.size()); ymc_=0.0;
    std::cout << "ymc_: " << std::endl << ymc_ << std::endl;
  }
};



#endif
