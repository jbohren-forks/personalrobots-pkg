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

#include <float.h>
#include <math.h>

typedef struct 
{
  string descriptor;
  NEWMAT::Matrix center;
  float theta;
  NEWMAT::Matrix vals;
  int id; //to identify which was learned first, second, third, etc.
} weak_classifier;

string displayWeakClassifier(const weak_classifier &wc); 

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
  map<int, unsigned int> num_class_objs_; //label, nExamples with that label.
  vector<int> classes_;

 DorylusDataset() : nClasses_(0)
  {
    version_string_ = std::string("#DORYLUS DATASET LOG v0.1");
  }

  std::string status();
  std::string displayFeatures();
  std::string displayYmc();
  void setObjs(const vector<object> &objs);
  bool save(std::string filename);
  bool load(std::string filename);
  std::string version_string_;
  bool testSave();
};


class Dorylus {
 public:
  map<string, vector<weak_classifier> > battery_;
  vector<weak_classifier*> pwcs_; //In the order that they were learned.
  NEWMAT::Matrix weights_; //nClasses x nTrEx.
  float objective_, objective_prev_, training_err_;
  DorylusDataset *dd_;
  string version_string_;
  int nWCs_;
  unsigned int nClasses_;
  vector<int> classes_;

  void loadDataset(DorylusDataset *dd);
  void normalizeWeights();
  bool learnWC(int nCandidates);
  float computeNewObjective(const weak_classifier& wc, const NEWMAT::Matrix& mmt, NEWMAT::Matrix** ppweights = NULL);
  float computeObjective();
  void train(int nCandidates, int max_secs, int max_wcs);
  bool save(string filename);
  bool load(string filename);
  vector<weak_classifier*> findActivatedWCs(const string &descriptor, const NEWMAT::Matrix &pt);
  NEWMAT::Matrix computeDatasetActivations(const weak_classifier& wc, const NEWMAT::Matrix& mmt);
  NEWMAT::Matrix classify(object &obj, NEWMAT::Matrix **confidence = NULL);
  float classify(DorylusDataset &dd);
  
 Dorylus() : dd_(NULL), nWCs_(0), nClasses_(0)
    {
      version_string_ = std::string("#DORYLUS CLASSIFIER LOG v0.1");
    }

};

class Function {
 public:
  Dorylus *d_;
  const NEWMAT::Matrix &mmt_;
  int label_;

 Function(Dorylus* d, const NEWMAT::Matrix &mmt, int label) : 
  d_(d), mmt_(mmt), label_(label)
  {
  }

  float operator()(float x) const {
    float val = 0.0;
    for(unsigned int m=0; m<d_->dd_->objs_.size(); m++) {
      val += d_->weights_(label_+1, m+1) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x);
    }
    return val;
  }

};


class Gradient : public Function {
 public:
 Gradient(Dorylus* d, const NEWMAT::Matrix &mmt, int label) : Function(d, mmt, label)
  {
  }

  float operator()(float x) const {
    float val = 0.0;
    for(unsigned int m=0; m<d_->dd_->objs_.size(); m++) {
      //      cout << "m " << m << endl;
/*       cout << d_->weights_.Nrows() << endl; */
/*       cout << d_->dd_->ymc_.Nrows() << endl; */
/*       cout << mmt_.Nrows() << endl; */
/*       cout << label_ << endl; */
      val += d_->weights_(label_+1, m+1) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x) * (-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1));
    }
    return val;
  }
};


class Hessian : public Function {
 public:

 Hessian(Dorylus* d, const NEWMAT::Matrix &mmt, int label) : Function(d, mmt, label)
  {
  }

  float operator()(float x) const {
    float val = 0.0;
    for(unsigned int m=0; m<d_->dd_->objs_.size(); m++) {
      val += d_->weights_(label_+1, m+1) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x) * (mmt_(1, m+1) * mmt_(1, m+1));
    }
    return val;
  }
};


float newtonSingle(const Function &fcn, const Gradient &grad, const Hessian &hes, float minDelta);
#endif
