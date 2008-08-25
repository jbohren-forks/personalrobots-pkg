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
#include <fstream>

typedef struct 
{
  string descriptor;
  NEWMAT::Matrix center;
  float theta;
  NEWMAT::Matrix vals;
} weak_classifier;

typedef struct 
{
  int label;
  map<string, NEWMAT::Matrix> features;
} object;

typedef float Real;

inline float euc(NEWMAT::Matrix a, NEWMAT::Matrix b)
{
  assert(a.Ncols() == 1 && b.Ncols() == 1);
  return (a-b).NormFrobenius();
}

class DorylusDataset {
 public:
  vector<object> objs_;
  NEWMAT::Matrix ymc_;
  unsigned int nClasses_;

 DorylusDataset(unsigned int nClasses) : nClasses_(nClasses)  
  {
    version_string_ = std::string("#DORYLUS DATASET LOG v0.1");
  }

  std::string status();
  std::string displayFeatures();
  void setObjs(const vector<object> &objs);
  bool save(std::string filename);
  bool load(std::string filename);
  std::string version_string_;
};


class Descriptor 
{
/*   virtual vector<float>* operator()(const SmartScan &ss, const IplImage &img, float x, float y, float z, float row, float col); */
 public:
  virtual void operator()()
  {}

};

class SpinLarge : public Descriptor
{
/*   vector<float>* operator()(const SmartScan &ss, const IplImage &img, float x, float y, float z, float row, float col) { */
/*     cout << "Computing large spin image." << endl; */
/*     return NULL; */
/*   } */
 public:
  void operator()()
  {
    std::cout << "Computing large spin image." << std::endl;
  }
};

class SpinMedium : public Descriptor
{
/*   vector<float>* operator()(const SmartScan &ss, const IplImage &img, float x, float y, float z, float row, float col) { */
/*     cout << "Computing medium spin image." << endl; */
/*     return NULL; */
/*   } */
 public:
  void operator()()
  {
    std::cout << "Computing medium spin image." << std::endl;
  }

};


#endif
