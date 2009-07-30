#ifndef DORYLUS_H
#define DORYLUS_H

//#include <Eigen/StdVector>
#include <limits>

#include <iomanip>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <Eigen/Core>
#include <Eigen/Array>

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
#include <cmath>




NEWMAT::Matrix* eigen2Newmat(const Eigen::MatrixXf& eig) {
  NEWMAT::Matrix *m = new NEWMAT::Matrix(eig.rows(), eig.cols());
  for(int i=0; i<eig.rows(); i++) {
    for(int j=0; j<eig.cols(); j++) {
      (*m)(i+1, j+1) = eig(i,j);
    }
  }
  return m;
}


typedef struct
{
  //! The name of the descriptor this weak classifier is concerned with.
  string descriptor;
  Eigen::MatrixXf center;
  float theta;
  //! The a_t^c's, i.e. the values of the weak classifier responses.
  NEWMAT::Matrix vals;
  //! to identify which was learned first, second, third, etc.
  int id;
  //! For centroid voting, boundary fragments, etc.  Not yet fully supported.
  map<string, void*> user_data;
  float utility;
} weak_classifier;

string displayWeakClassifier(const weak_classifier &wc);

class object {
 public:
  //! 0 = Background.
  int label;
  map<string, Eigen::MatrixXf*> features;


  string status(bool showFeatures=true);
  object() {}
  ~object() {
    map<string, Eigen::MatrixXf*>::iterator fit;
    for(fit=features.begin(); fit!=features.end(); fit++) {
      delete fit->second;
    }
  }

  //! Allocates new memory for features.
  object(const object& o) {
    label = o.label;
    map<string, Eigen::MatrixXf*>::const_iterator fit; 
    for(fit = o.features.begin(); fit!=o.features.end(); fit++) {
      features[fit->first] = new Eigen::MatrixXf(*(fit->second));
    }
  }    
};


inline float euc(const NEWMAT::Matrix& a, const NEWMAT::Matrix& b)
{
  assert(a.Ncols() == 1 && b.Ncols() == 1);
  assert(a.Nrows() == b.Nrows());

  NEWMAT::Real* pa = a.Store();
  NEWMAT::Real* pb = b.Store();
  float r = 0;
  for(int i=0; i<a.Nrows(); i++) {
    r += pow((float)((*pa) - (*pb)), 2);
    pa++;
    pb++;
  }

  //assert(abs((float)((a-b).NormFrobenius() - sqrt(r))) < 1e-3);  
  return sqrt(r);
}

inline float euc(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b)
{
  assert(a.cols() == 1 && b.cols() == 1);
  assert(a.rows() == b.rows());

  float r = 0;
  for(int i=0; i<a.rows(); i++) {
    r += pow(a.coeff(i,0) - b.coeff(i,0), 2);
  }

  //assert(abs((float)((a-b).NormFrobenius() - sqrt(r))) < 1e-3);  
  return sqrt(r);

  //Slow
/*   Eigen::MatrixXf c = (a - b).cwise().pow(2); */
/*   return sqrt(c.sum()); */
}


class DorylusDataset {
 public:
  vector<object*> objs_;
  //! NEWMAT::Matrix of y_m^c values.  i.e. ymc_(c,m) = +1 if the label of training example m is c, and -1 otherwise.
  NEWMAT::Matrix ymc_;
  //! Number of classes, not including class 0.
  unsigned int nClasses_; 
  //! num_class_objs[label] = num training examples with that label.  Unlike classes_, this does include the label 0, which is used for background / unknown objects.
  map<int, unsigned int> num_objs_of_class_;
  //! List of labels in the dataset.  Does not include the label 0, which is used for background / unknown objects.
  vector<int> classes_;
  std::string version_string_;

 DorylusDataset() : nClasses_(0)
  {
    version_string_ = std::string("#DORYLUS DATASET LOG v0.1");
    cout << "DorylusDataset(), version_string_ = "<< version_string_ << endl;
  }

  std::string status();
  std::string displayObjects();
  std::string displayYmc();

  void setObjs(const vector<object*> &objs);
  bool save(std::string filename);
  bool load(std::string filename, bool quiet=false);
  bool join(const DorylusDataset& dd2);
  bool compare(const DorylusDataset& dd);
};


class Dorylus {
 public:
  //! Weak classifiers are stored according to which descriptor they are from.
  map<string, vector<weak_classifier*> > battery_;
  //! Pointers to weak classifiers, in the order that they were learned.
  vector<weak_classifier*> pwcs_;
  //! nClasses x nTrEx.
  NEWMAT::Matrix log_weights_;
  float objective_, objective_prev_, training_err_;
  DorylusDataset *dd_;
  string version_string_;
  unsigned int nClasses_;
  vector<int> classes_;
  vector<string> exclude_descriptors_;
  //! Prevents classifier from using any weak classifier learned after this number of weak classifiers.  If 0, no limit.
  int max_wc_;

  //! debugHook will be called each time a new weak classifier is learned. 
  void train(int nCandidates, int max_secs, int max_wcs, void (*debugHook)(weak_classifier)=NULL);
  void useDataset(DorylusDataset *dd);
  bool save(string filename, string *user_data_str=NULL);
  bool load(string filename, bool quiet=false, string *user_data_str=NULL);
  std::string status();
  bool learnWC(int nCandidates, map<string, float> max_thetas, vector<string> *desc_ignore=NULL);
  NEWMAT::Matrix classify(object &obj, NEWMAT::Matrix **confidence = NULL);
  float classify(DorylusDataset &dd);
  map<string, float> computeMaxThetas(const DorylusDataset &dd);
  //  float computeNewObjective(const weak_classifier& wc, const NEWMAT::Matrix& mmt, NEWMAT::Matrix** ppweights = NULL);
  float computeUtility(const weak_classifier& wc, const NEWMAT::Matrix& mmt);
  float computeObjective();
  //void train(int nCandidates, int max_secs, int max_wcs);
  vector<weak_classifier*>* findActivatedWCs(const string &descriptor, const Eigen::MatrixXf &pt);
  NEWMAT::Matrix computeDatasetActivations(const weak_classifier& wc, const NEWMAT::Matrix& mmt);
  bool compare(const Dorylus& d);

 Dorylus() : dd_(NULL), nClasses_(0), max_wc_(0)
    {
      version_string_ = std::string("#DORYLUS CLASSIFIER LOG v0.2");
    }

  ~Dorylus() {
    for(size_t i=0; i<pwcs_.size(); ++i) {
      delete pwcs_[i];
    }
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
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x);
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
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x) * (-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1));
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
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(1, m+1) * x) * (mmt_(1, m+1) * mmt_(1, m+1));
    }
    return val;
  }
};


float newtonSingle(const Function &fcn, const Gradient &grad, const Hessian &hes, float minDelta);
#endif
