/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alex Teichman
*********************************************************************/

#ifndef DORYLUS_H
#define DORYLUS_H

#include <limits>

#include <iomanip>
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
#include <string>

typedef struct
{
  //! The name of the descriptor this weak classifier is concerned with.
  std::string descriptor;
  Eigen::MatrixXf center;
  float theta;
  //! The a_t^c's, i.e. the values of the weak classifier responses.
  Eigen::VectorXf vals;
  //! to identify which was learned first, second, third, etc.
  int id;
  //! For centroid voting, boundary fragments, etc.  Not yet fully supported.
  std::map<std::string, void*> user_data;
  float utility;
} weak_classifier;

std::string displayWeakClassifier(const weak_classifier &wc);

class object {
 public:
  //! 0 = Background.
  int label;
  std::map<std::string, Eigen::MatrixXf*> features;


  std::string status(bool showFeatures=true);
  object() {}
  ~object() {
    std::map<std::string, Eigen::MatrixXf*>::iterator fit;
    for(fit=features.begin(); fit!=features.end(); fit++) {
      delete fit->second;
    }
  }

  //! Allocates new memory for features.
  object(const object& o) {
    label = o.label;
    std::map<std::string, Eigen::MatrixXf*>::const_iterator fit; 
    for(fit = o.features.begin(); fit!=o.features.end(); fit++) {
      features[fit->first] = new Eigen::MatrixXf(*(fit->second));
    }
  }    
};


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
  std::vector<object*> objs_;
  //! Matrix of y_m^c values.  i.e. ymc_(c,m) = +1 if the label of training example m is c, and -1 otherwise.
  Eigen::MatrixXf ymc_;
  //! Number of classes, not including class 0.
  unsigned int nClasses_; 
  //! num_class_objs[label] = num training examples with that label.  Unlike classes_, this does include the label 0, which is used for background / unknown objects.
  std::map<int, unsigned int> num_objs_of_class_;
  //! List of labels in the dataset.  Does not include the label 0, which is used for background / unknown objects.
  std::vector<int> classes_;
  std::string version_string_;

 DorylusDataset() : nClasses_(0)
  {
    version_string_ = std::string("#DORYLUS DATASET LOG v0.1");
    std::cout << "DorylusDataset(), version_string_ = "<< version_string_ << std::endl;
  }

  std::string status();
  std::string displayObjects();
  std::string displayYmc();

  void setObjs(const std::vector<object*> &objs);
  bool save(std::string filename);
  bool load(std::string filename, bool quiet=false);
  bool join(const DorylusDataset& dd2);
  bool compare(const DorylusDataset& dd);
};


class Dorylus {
 public:
  //! Weak classifiers are stored according to which descriptor they are from.
  std::map<std::string, std::vector<weak_classifier*> > battery_;
  //! Pointers to weak classifiers, in the order that they were learned.
  std::vector<weak_classifier*> pwcs_;
  //! nClasses x nTrEx.
  Eigen::MatrixXf log_weights_;
  float objective_, objective_prev_, training_err_;
  DorylusDataset *dd_;
  std::string version_string_;
  unsigned int nClasses_;
  std::vector<int> classes_;
  std::vector<std::string> exclude_descriptors_;
  //! Prevents classifier from using any weak classifier learned after this number of weak classifiers.  If 0, no limit.
  int max_wc_;

  //! debugHook will be called each time a new weak classifier is learned. 
  void train(int nCandidates, int max_secs, int max_wcs, void (*debugHook)(weak_classifier)=NULL);
  void useDataset(DorylusDataset *dd);
  bool save(std::string filename, std::string *user_data_str=NULL);
  bool load(std::string filename, bool quiet=false, std::string *user_data_str=NULL);
  std::string status();
  bool learnWC(int nCandidates, std::map<std::string, float> max_thetas, std::vector<std::string> *desc_ignore=NULL);
  Eigen::VectorXf classify(object &obj);
  float classify(DorylusDataset &dd);
  std::map<std::string, float> computeMaxThetas(const DorylusDataset &dd);
  float computeUtility(const weak_classifier& wc, const Eigen::VectorXf& mmt);
  float computeObjective();
  //void train(int nCandidates, int max_secs, int max_wcs);
  std::vector<weak_classifier*>* findActivatedWCs(const std::string &descriptor, const Eigen::MatrixXf &pt);
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
  const Eigen::MatrixXf &mmt_;
  int label_;

 Function(Dorylus* d, const Eigen::MatrixXf &mmt, int label) :
  d_(d), mmt_(mmt), label_(label)
  {
  }

  float operator()(float x) const {
    float val = 0.0;
    for(unsigned int m=0; m<d_->dd_->objs_.size(); m++) {
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(0, m) * x);
    }
    return val;
  }

};


class Gradient : public Function {
 public:
 Gradient(Dorylus* d, const Eigen::MatrixXf &mmt, int label) : Function(d, mmt, label)
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
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(0, m) * x) * (-d_->dd_->ymc_(label_+1, m+1) * mmt_(0, m));
    }
    return val;
  }
};


class Hessian : public Function {
 public:

 Hessian(Dorylus* d, const Eigen::MatrixXf &mmt, int label) : Function(d, mmt, label)
  {
  }

  float operator()(float x) const {
    float val = 0.0;
    for(unsigned int m=0; m<d_->dd_->objs_.size(); m++) {
      val += exp(d_->log_weights_(label_+1, m+1)) * exp(-d_->dd_->ymc_(label_+1, m+1) * mmt_(0, m) * x) * (mmt_(0, m) * mmt_(0, m));
    }
    return val;
  }
};


float newtonSingle(const Function &fcn, const Gradient &grad, const Hessian &hes, float minDelta);
#endif
