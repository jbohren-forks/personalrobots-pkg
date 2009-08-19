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
#include <dorylus/dorylus.h>
#include <signal.h>


using namespace std;
USING_PART_OF_NAMESPACE_EIGEN
#define EIGEN_NO_DEBUG

bool g_int = false;

float newtonSingle(const Function &fcn, const Gradient &grad, const Hessian &hes, float minDelta) {
  float delta = 1e10;
  float x = 0.0, x_old=0.0;
  while(fabs(delta) > minDelta) {
    x_old = x;
    x = x - (grad(x) / hes(x));
    delta = x - x_old;

    cout << "obj: " << fcn(x) << "  delta: " << delta << "  x: " << x << "  x_old: " << x_old << endl;
    usleep(1000);
  }
  cout << "DONE" << endl;
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return x;
}

bool DorylusDataset::join(const DorylusDataset& dd2)
{
  // -- Copy dd2's objects.
  vector<object*> objs = objs_;
  const vector<object*>& objs2 = dd2.objs_;
  for(unsigned int i=0; i<objs2.size(); i++) {
    objs.push_back(new object(*(objs2[i])));
  }   
  // -- Recompute ymc_, nClasses_, num_objs_of_class_, and classes_.
  setObjs(objs);
  return true;
}

bool DorylusDataset::compare(const DorylusDataset& dd) {
  for(size_t i=0; i<objs_.size(); ++i) {
    object& o = *objs_[i];
    object& o2 = *dd.objs_[i];
    if(o.label != o2.label)
      return false;
    if(o.features.size() != o2.features.size())
      return false;

    map<string, MatrixXf*>::const_iterator it;
    for(it = o.features.begin(); it != o.features.end(); ++it) {
      if(*o2.features[it->first] != *it->second)
	return false;
    }
  }
    
  return true;
}


void DorylusDataset::setObjs(const vector<object*> &objs)
{
  //num_objs_of_class_ contains the number of bg objs.  classes_ does NOT, nor does nClasses_
  objs_ = objs;
  num_objs_of_class_.clear();
  classes_.clear();

  // -- Get num_objs_of_class_, classes_, and nClasses_.
  for(unsigned int m=0; m<objs.size(); m++) {
    // Don't count label=0 as a class!
    if(num_objs_of_class_.find(objs[m]->label) == num_objs_of_class_.end() && objs[m]->label != 0)
      classes_.push_back(objs[m]->label);
    num_objs_of_class_[objs[m]->label] = 0;
  }
  for(unsigned int m=0; m<objs.size(); m++) {
    num_objs_of_class_[objs[m]->label]++;
  }
  nClasses_ = classes_.size();


  // -- Construct ymc_
  ymc_ = MatrixXf::Zero(nClasses_, objs.size());
  for(unsigned int m=0; m<objs.size(); m++) {
    for(unsigned int c=0; c<nClasses_; c++) {
      //If the label is 0 (BG), make all y_m^c's be -1.
      if(objs_[m]->label == (int)classes_[c] && objs_[m]->label != 0) {
	ymc_(c,m) = 1;
      }
      else
	ymc_(c,m) = -1;
    }
  }
}

string DorylusDataset::displayYmc() {
  ostringstream oss (ostringstream::out);
  oss << "ymc_: " << endl << ymc_ << endl;
  return oss.str();
}

string DorylusDataset::displayObjects() {
  ostringstream oss (ostringstream::out);
  for(unsigned int i=0; i<objs_.size(); i++) {
    oss << "Object " << i << " " << endl;
    oss << objs_[i]->status();
  }
  return oss.str();
}

string object::status(bool showFeatures) {
  map<string, MatrixXf*>::const_iterator it;
  ostringstream oss (ostringstream::out);
  oss << "Object with label " << label << ":" << endl;
  for(it = features.begin(); it!=features.end(); it++) {
    MatrixXf* v = it->second;
    oss << it->first << " descriptor (" << v->rows() << " dimensions) " << endl;
    if(showFeatures)
      oss << v->transpose() << endl;
  }
  return oss.str();
}

std::string Dorylus::status()
{
  char tmp[1000];
  string st("Classifier Status: \n");
  st.append("  nClasses: ");
  sprintf(tmp, "%d \n", nClasses_);
  st.append(tmp);

  st.append("  nWeakClassifier: ");
  sprintf(tmp, "%zd \n", pwcs_.size());
  st.append(tmp);

  if(pwcs_.size() == 0)
    return st;

  // -- Compute utilities for each descriptor space.
  map<string, float> utilities;
  map<string, vector<weak_classifier*> >::iterator it;
  for(it=battery_.begin(); it != battery_.end(); it++) {
    vector<weak_classifier*>& vwc = it->second;
    for(size_t i=0; i<vwc.size(); i++) {
      utilities[it->first] += vwc[i]->utility;
    }
  }

  sprintf(tmp, "  nWeakClassifiers \t Utility \t Name \n");
  st.append(tmp);

  // -- Print in order of best utility.
  while(utilities.size() > 0) {
    // -- Find the descriptor space with the best util.
    map<string, float>::iterator uit;
    float max = 0;
    string name = "";
    for(uit=utilities.begin(); uit != utilities.end(); uit++) {
      if(uit->second > max) {
	max = uit->second;
	name = uit->first;
      }
    }
      
    sprintf(tmp, "  %zd \t\t\t %02.5f \t %s \n", battery_[name].size(), utilities[name], name.c_str());  
    st.append(tmp);
    utilities.erase(name);
  }
 
  return st;
}



std::string DorylusDataset::status()
{
  ostringstream oss (ostringstream::out);

  oss << "DorylusDataset status: \n";
  if(objs_.size() < 1) {
    oss << "No objects!" << "\n";
    return oss.str();
  }
  
  oss << "  nClasses: " << nClasses_ << "\n";
  oss << "  nObjects: " << objs_.size() << "\n";
  oss << "  nDescriptors: " << objs_[0]->features.size() << "\n";
  oss << "  num_objs_of_class_: " << "\n";
  for(map<int, unsigned int>::iterator it=num_objs_of_class_.begin(); it != num_objs_of_class_.end(); it++) {
    oss << "    class " << (*it).first << ": " << (*it).second << " objects. \n";
  }

  map<string, MatrixXf*>::iterator fit;
  map<string, int> nPts, dim;
  for(unsigned int m=0; m<objs_.size(); m++) {
    for(fit = objs_[m]->features.begin(); fit != objs_[m]->features.end(); fit++) {
      nPts[fit->first] += fit->second->cols();
      if(fit->second->rows() != 0)
	dim[fit->first] = fit->second->rows();
    }
  }

  oss << "  nPts: " << endl;
  map<string, int>::iterator pit;
  for(pit = nPts.begin(); pit != nPts.end(); pit++) {
    oss << "    nPts in " << pit->first << " (" << dim[pit->first] << "-dimensional) space: " << pit->second << "\n";
  }

  // -- Show example features.
//   for(fit = objs_[0]->features.begin(); fit != objs_[0]->features.end(); fit++) {
//     cout << "   " << fit->first << ": " << endl << fit->second->transpose() << endl;
//   }


  return oss.str();
}

bool DorylusDataset::save(string filename)
{
  ofstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  f << version_string_ << endl;
  float buf;
  for(unsigned int i=0; i<objs_.size(); i++) {
    object* obj = objs_[i];
    f << "New object." << endl;
    f << obj->label << endl;
    map<string, MatrixXf*>::iterator it;
    for(it = obj->features.begin(); it!=obj->features.end(); it++) {
      MatrixXf* v = it->second;
      f << "New descriptor." << endl;
      f << it->first << endl;
      f << v->rows() << endl;
      f << v->cols() << endl;
      for(int k=0; k<v->cols(); k++) {
	for(int m=0; m<v->rows(); m++) {
	  buf = (*v)(m,k);
	  f.write((char*)&buf, sizeof(float));
	}
      }
      f << endl;
    }
  }
  f.close();
  return true;
}

bool DorylusDataset::load(string filename, bool quiet)
{
  ifstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  string line;
  getline(f, line);
  if(line.compare(version_string_) != 0) {
    cerr << "Log " << filename << " is of the wrong type!" << endl;
    return false;
  }

  cout << "Loading " << filename << endl;

  vector<object*> objs;
  int nRows, nCols;
  string descriptor;
  object *pobj = NULL;
  while(true) {

    getline(f, line);

    if(line.size() == 0) {
      if(pobj) {
	objs.push_back(pobj);
	pobj = NULL;
      }
      else {
	cerr << "no object??" << endl;
      }
      break;
    }

    else if(line.compare(string("New object.")) == 0) {
      if(pobj) {
	objs.push_back(pobj);
	pobj = NULL;
      }
      pobj = new object();
      getline(f, line);
      istringstream iss_label(line);
      iss_label >> pobj->label;
    }

    else if(line.compare(string("New descriptor.")) == 0) {
      getline(f, descriptor);
      getline(f, line);
      istringstream iss_nrows(line);
      iss_nrows >> nRows;
      getline(f, line);
      istringstream iss_ncols(line);
      iss_ncols >> nCols;

      float* pbuf = (float*)malloc(sizeof(float)*nRows*nCols);
      f.read((char*)pbuf, sizeof(float)*nRows*nCols); 
      MatrixXf* tmp = new MatrixXf();
      *tmp = Eigen::Map<MatrixXf>(pbuf, nRows, nCols);
      pobj->features[descriptor] = tmp;


//       float buf;
//       pobj->features[descriptor] = new MatrixXf(nRows, nCols);
//       for(int i=0; i<nCols; i++) {
// 	for (int j=0; j<nRows; j++) {
// 	  f.read((char*)&buf, sizeof(float)); 
// 	  (*(pobj->features[descriptor]))(j, i) = buf;
// 	}
//       }


      getline(f, line); //Move off the line with the data.
    }

    else {
      cerr << "Error reading log.  Line: " << line << endl;
      return false;
    }
  }

  f.close();
  setObjs(objs);
  cout << "Done loading " << filename << endl;
  return true;
}


bool Dorylus::save(string filename, string *user_data_str)
{
  ofstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  f << version_string_ << endl;
  f << "Classes." << endl;
  for(unsigned int i=0; i<classes_.size(); i++) {
    f << classes_[i] << endl;
  }
  f << "End classes." << endl;

  float fbuf;
  double dbuf;
  map<string, vector<weak_classifier*> >::iterator bit;
  for(bit = battery_.begin(); bit != battery_.end(); bit++) {
    vector<weak_classifier*> &wcs = bit->second;
    for(unsigned int t=0; t<wcs.size(); t++) {
      f << "New weak classifier." << endl;
      f << "ID" << endl << wcs[t]->id << endl;
      f << bit->first << endl;
      f << "Theta" << endl << wcs[t]->theta << endl;
      f << "Utility" << endl << wcs[t]->utility << endl;
      f << "Center" << endl << wcs[t]->center.rows() << endl;
      for(int i=0; i<wcs[t]->center.rows(); i++) {
	fbuf = wcs[t]->center(i,0);
	f.write((char*)&fbuf, sizeof(float));
      }
      f << endl;
      f << "Vals" << endl << wcs[t]->vals.rows() << endl;
      for(int i=0; i<wcs[t]->vals.rows(); ++i) {
	fbuf = wcs[t]->vals(i);
	f.write((char*)&fbuf, sizeof(float));
      }
      f << endl;
      f << "Numerators" << endl << wcs[t]->numerators.rows() << endl;
      for(int i=0; i<wcs[t]->numerators.rows(); ++i) {
	dbuf = wcs[t]->numerators(i);
	f.write((char*)&dbuf, sizeof(double));
      }
      f << endl;
      f << "Denominators" << endl << wcs[t]->denominators.rows() << endl;
      for(int i=0; i<wcs[t]->denominators.rows(); ++i) {
	dbuf = wcs[t]->denominators(i);
	f.write((char*)&dbuf, sizeof(double));
      }
      f << endl;
    }
  }
  f << "User data." << endl;
  if(user_data_str)
    f << *user_data_str << endl;
  f << "End user data." << endl;

  f.close();
  return true;
}


bool Dorylus::load(string filename, bool quiet, string *user_data_str)
{
  ifstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  string line;
  getline(f, line);
  if(line.compare(version_string_) != 0) {
    cerr << "Log " << filename << " is of the wrong type!" << endl;
    cerr << "First line is: " << line << " instead of " << version_string_ << endl;
    return false;
  }

  cout << "Loading " << filename << endl;

  // -- Read the classes.
  getline(f, line);
  assert(line.compare("Classes.") == 0);
  while(true) {
    getline(f, line);
    if(line.compare("End classes.") == 0)
      break;
    istringstream iss_class(line);
    int c;
    iss_class >> c;
    classes_.push_back(c);
  }


  battery_.clear();
  string descriptor;
  weak_classifier *pwc = NULL;
  while(true) {

    getline(f, line);

    if(line.size() == 0) {
      if(pwc) {
	battery_[pwc->descriptor].push_back(pwc);
	pwcs_.push_back(pwc);
	pwc = NULL;
      }
      else {
	cerr << "no wc??" << endl;
      }
      break;
    }

    else if(line.compare(string("User data.")) == 0) {
      if(user_data_str != NULL)
	*user_data_str = string("");
      while(true) {
	getline(f, line);
	if(line.compare(string("End user data.")) == 0 || line.size() == 0)
	  break;
	if(user_data_str != NULL)
	  *user_data_str = *user_data_str + string("\n") + line;
      }
    }

    else if(line.compare(string("New weak classifier.")) == 0) {
      if(pwc) {
	battery_[pwc->descriptor].push_back(pwc);
	pwcs_.push_back(pwc);
	pwc = NULL;
      }
      pwc = new weak_classifier;
      getline(f, line);
      assert(line.compare("ID") == 0);
      getline(f, line);
      istringstream iss_id(line);
      iss_id >> pwc->id;

      getline(f, pwc->descriptor);

      getline(f, line);
      assert(line.compare("Theta") == 0);
      getline(f, line);
      istringstream iss_theta(line);
      iss_theta >> pwc->theta;

      getline(f, line);
      assert(line.compare("Utility") == 0);
      getline(f, line);
      istringstream iss_util(line);
      iss_util >> pwc->utility;

      getline(f, line);
      assert(line.compare("Center") == 0);
      getline(f, line);
      istringstream iss_nEle(line);
      int nElements;
      iss_nEle >> nElements;
      float fbuf;
      pwc->center = MatrixXf(nElements, 1);
      for(int i=0; i<nElements; i++) {
	f.read((char*)&fbuf, sizeof(float));
	pwc->center(i,0) = fbuf;
      }
      getline(f, line); //Move off the line with the data.

      getline(f, line);
      if(line.compare("Vals") != 0)
	cout << line << " is not vals!" << endl;
      assert(line.compare("Vals") == 0);
      getline(f, line);
      istringstream iss_nEleVals(line);
      iss_nEleVals >> nClasses_;
      pwc->vals = VectorXf::Zero(nClasses_);
      for(unsigned int i=0; i<nClasses_; ++i) {
	f.read((char*)&fbuf, sizeof(float));
	pwc->vals(i) = fbuf;
      }
      getline(f, line); //Move off the line with the data.

      // -- Numerators
      double dbuf;
      getline(f, line);
      assert(line.compare("Numerators") == 0);
      getline(f, line); // This line has nClasses_ but we already know it.
      pwc->numerators = VectorXd::Zero(nClasses_);
      for(unsigned int i=0; i<nClasses_; ++i) {
	f.read((char*)&dbuf, sizeof(double));
	pwc->numerators(i) = dbuf;
      }
      getline(f, line); //Move off the line with the data.

      // -- Denominators
      getline(f, line);
      assert(line.compare("Denominators") == 0);
      getline(f, line);
      pwc->denominators = VectorXd::Zero(nClasses_);
      for(unsigned int i=0; i<nClasses_; ++i) {
	f.read((char*)&dbuf, sizeof(double));
	pwc->denominators(i) = dbuf;
      }
      getline(f, line); //Move off the line with the data.
    }

    else {
      cerr << "Error reading log.  Line: " << line << endl;
      return false;
    }
  }

  f.close();
  cout << "Done loading " << filename << endl;
  return true;
}


void Dorylus::useDataset(DorylusDataset *dd) {
  dd_ = dd;

  log_weights_ = MatrixXf::Zero(dd_->nClasses_, dd_->objs_.size());

  nClasses_ = dd_->nClasses_;
  classes_ = dd->classes_;
}

vector<weak_classifier*>* Dorylus::findActivatedWCs(const string &descriptor, const MatrixXf &pt) {
  vector<weak_classifier*> &wcs = battery_[descriptor];
  vector<weak_classifier*> *activated = new vector<weak_classifier*>;
  activated->reserve(wcs.size());

  for(unsigned int t=0; t<wcs.size(); t++) {
    if(max_wc_ == 0 || max_wc_ > 0 && wcs[t]->id <= max_wc_) {
      if(euc(pt, wcs[t]->center) <= wcs[t]->theta) {
	activated->push_back(wcs[t]);
      }
    }
  }
  return activated;
}


void sigint(int none) {
  cout << "Caught user signal." << endl;
  g_int = true;
}

void Dorylus::train(int num_candidates, int max_secs, int max_wcs, double min_util, void (*debugHook)(weak_classifier)) {
  signal(SIGINT,sigint);
  time_t start, end;
  time(&start);
  float obj, obj2;

  obj = computeObjective();
  cout << "Objective: " << obj << endl;
  cout << "Objective (from classify()): " << classify(*dd_) << endl;
    	  
  int wcs=0;
  while(true) {
    learnWC(num_candidates);
    wcs++;
    time(&end);
    if(debugHook != NULL)
      debugHook(*pwcs_.back());

    obj2 = computeObjective();
    cout << "Objective: " << obj2 << endl;

    assert(obj2 < obj);

    if(max_secs != 0 && difftime(end,start) > max_secs) {
      cout << "Ending training because max time has been reached." << endl;
      break;
    }
    if(max_wcs != 0 && wcs >= max_wcs) {
      cout << "Ending training because max number of weak classifiers has been reached." << endl;
      break;
    }
    if(g_int) {
      cout << "Ending training because of user control-c." << endl;
      break;
    }
    if(pwcs_.back()->utility < min_util) {
      cout << "Ending training because of min utility criterion: " << pwcs_.back()->utility << " < min util of " << min_util << endl;
      break;
    }
  }

  cout << "Done training." << endl;
}

//! Comprehensive with high probability, but not guaranteed.
bool Dorylus::compare(const Dorylus& d, bool verbose) {
  if(pwcs_.size() != d.pwcs_.size()) {
    if(verbose)
      cout << "Different number of weak classifiers!" << endl;
    return false;
  }
    


  for(size_t i=0; i<pwcs_.size(); ++i) {
//     cout << i << " theta " << pwcs_[i]->theta << " " <<  d.pwcs_[i]->theta << endl;
//     cout << i << " utility " << pwcs_[i]->utility << " " <<  d.pwcs_[i]->utility << endl;
//     cout << i << " id " << pwcs_[i]->id << " " <<  d.pwcs_[i]->id << endl;

    if(abs(pwcs_[i]->theta - d.pwcs_[i]->theta) > 1e-4) {
      if(verbose)
	cout << "thetas for wc " << i << ": " << pwcs_[i]->theta << ", " << d.pwcs_[i]->theta << endl;
      return false;
    }
    if(abs(pwcs_[i]->utility - d.pwcs_[i]->utility) > 1e-4) {
      if(verbose)
	cout << "utilities for wc " << i << ": " << pwcs_[i]->utility << ", " << d.pwcs_[i]->utility << endl;
      return false;
    }
    if(abs(pwcs_[i]->id - d.pwcs_[i]->id) > 1e-4) {
      if(verbose)
	cout << "ids for wc " << i << ": " << pwcs_[i]->id << ", " << d.pwcs_[i]->id << endl;
      return false;
    }
  }

  return true;
}

vector<weak_classifier*> Dorylus::createRandomWeakClassifiers(int num_candidates) {
  // -- Get a weights matrix that does not include the bg pts.
  map<int, unsigned int>:: iterator it = dd_->num_objs_of_class_.begin();
  int nNonBGObjs = 0;
  for(; it != dd_->num_objs_of_class_.end(); it++) {
    if(it->first == 0)
      continue;
    nNonBGObjs += it->second;
  }
  MatrixXf non_bg_weights = MatrixXf::Zero(nClasses_, nNonBGObjs);
  int m=0;
  for(int col=0; col<nNonBGObjs; col++) {
    while(dd_->objs_[m]->label==0) {
      m++;
    }
    non_bg_weights.col(col) = log_weights_.col(m);
    m++;
  }
  non_bg_weights = non_bg_weights.cwise().exp();

  // -- Choose wc candidates from the distribution of weights over the non-background objects.
  vector<weak_classifier*> cand;
  for(int iCand=0; iCand<num_candidates; iCand++) {
    double dice = (double)rand() / (double)RAND_MAX * non_bg_weights.sum(); //The weights aren't necessarily normalized.
    double w = 0.0;
    int obj_id=-1;
    int m=0;
    for(int i=0; i<non_bg_weights.cols(); i++, m++) {
      while(dd_->objs_[m]->label == 0)
	m++;
      w = non_bg_weights.col(i).sum();
      dice -= w;
      if(dice <= 0) {
	obj_id = m;
	break;
      }
    }
    if(obj_id == -1) {
      cerr << "**** obj id = -1." << endl;
      obj_id = 0;
    }
    assert(obj_id >= 0);

    // -- Get a random descriptor space from the object.
    map<string, MatrixXf*> &ft = dd_->objs_[obj_id]->features;
    MatrixXf* v;
    string desc;
    int desc_id = rand() % (int)ft.size();

    map<string, MatrixXf*>::iterator it = ft.begin();
    for(int i=0; i<desc_id; i++) {
      it++;
    }
    v = (*it).second;
    desc = (*it).first;
    
    //Get a random feature.
    int feature_id = (rand() % (int)v->cols());

    //Make the weak classifier.
    weak_classifier* wc = new weak_classifier;
    wc->descriptor = desc;
    wc->center = v->col(feature_id);
    wc->theta = 0;
    wc->vals = VectorXf::Zero(dd_->nClasses_);
    wc->id = -1;
    wc->utility = 0;
    wc->numerators = VectorXd::Zero(nClasses_);
    wc->denominators = VectorXd::Zero(nClasses_);
    cand.push_back(wc);
  }
  return cand;
}

vector< pair<double, int> >* DorylusDataset::computeSortedDistances(string descriptor, VectorXf center) {
  vector< pair<double, int> >* distance_idx = new vector< pair<double, int> >(objs_.size());
  for(size_t j=0; j<objs_.size(); ++j) {
    MatrixXf* f = objs_[j]->features[descriptor];
    assert(f->cols() == 1);
    float dist = euc(*f, center);
    (*distance_idx)[j].first = dist;
    (*distance_idx)[j].second = j; 
  }
  sort(distance_idx->begin(), distance_idx->end());

  return distance_idx;
}

void Dorylus::learnWC(int num_candidates) {
  assert(dd_);

  time_t start, end;
  time(&start);

  vector<weak_classifier*> cand = createRandomWeakClassifiers(num_candidates);
  cout << "Added " << num_candidates << " candidate wcs" << endl;

  vector<object*>& objs = dd_->objs_;
  weak_classifier* best_wc = new weak_classifier;
  double max_utility = 0;
  vector< pair<double, int> >* best_distance_idx = NULL;
  int num_tr_ex_in_best = 0;
  for(unsigned int i=0; i<cand.size(); ++i) {
    // -- Get all distances and sort.
    vector< pair<double, int> >* distance_idx = dd_->computeSortedDistances(cand[i]->descriptor, cand[i]->center);  // (*distance_idx)[j] is distance, idx (i.e. objs_[idx])

    // -- Setup vars for evaluating weak classifiers efficiently.
    vector<double> weight_sum_pos(nClasses_); // sum of weight inside hypersphere for each class
    vector<double> weight_sum_neg(nClasses_); // sum of weight inside hypersphere for all y_m^c == -1

    // -- For all training examples in order of distance.
    for(size_t m=0; m<distance_idx->size(); ++m) {
      if(m>0)
	assert((*distance_idx)[m-1].first <= (*distance_idx)[m].first);

      int idx = (*distance_idx)[m].second; // object idx of the mth closest training example
      double utility = 0;
      for(size_t c=0; c<nClasses_; ++c) {
	// -- Update the response.
	cand[i]->denominators(c) += exp(log_weights_(c,idx));
	cand[i]->numerators(c) += exp(log_weights_(c,idx)) * dd_->ymc_(c,idx);
	if(cand[i]->denominators(c) == 0)
	  cand[i]->vals(c) = 0;
	else
	  cand[i]->vals(c) = cand[i]->numerators(c) / cand[i]->denominators(c);

	// -- Update the sum of weights in the hypersphere.
	assert(dd_->ymc_(c,idx) == 1 || dd_->ymc_(c,idx) == -1);
	if(dd_->ymc_(c,idx) == 1) 
	  weight_sum_pos[c] += exp(log_weights_(c,idx));
	else
	  weight_sum_neg[c] += exp(log_weights_(c,idx));
	
	utility += ((1-exp(-cand[i]->vals(c))) * weight_sum_pos[c] + (1-exp(cand[i]->vals(c))) * weight_sum_neg[c]) / (objs.size() * nClasses_);
      }
      // Doing the division here makes 32bit vs 64bit give different answers.
      // utility /= objs.size() * nClasses_;

      // -- If this is the best so far, make a copy.
      if(utility > max_utility) {
	max_utility = utility;
	cand[i]->theta = (*distance_idx)[m].first; // Set theta to be exactly the distance to this training example.
	cand[i]->utility = utility;
	*best_wc = *cand[i];

	if(!best_distance_idx)
	  best_distance_idx = distance_idx;
	if(best_distance_idx != distance_idx) {
	  delete best_distance_idx;
	  best_distance_idx = distance_idx;
	}
	num_tr_ex_in_best = m;
      }
    }
    cout << "."; cout.flush();
  }
  cout << endl;
  
  // -- Add the new weak classifier.
  cout << "Found weak classifier with utility " << max_utility << endl;
  assert(max_utility > 0);
  best_wc->id = pwcs_.size()+1;
  battery_[best_wc->descriptor].push_back(best_wc);
  pwcs_.push_back(best_wc);
  
  // -- Compute the new log weights.
  for(int m=0; m<num_tr_ex_in_best; ++m) {
    int idx = (*best_distance_idx)[m].second;
    for(unsigned int c=0; c<dd_->nClasses_; ++c) {
      log_weights_(c, idx) += -dd_->ymc_(c, idx) * best_wc->vals(c);
    }
  }


  // -- Display stats about the weak classifier.
  time(&end);
  cout << "Took " << difftime(end,start) << " seconds to try " << num_candidates << " wcs." << endl;
  cout << displayWeakClassifier(*best_wc) << endl;
  cout << "WC encompasses at least one point from " << num_tr_ex_in_best << " out of " << objs.size() << " objects." << endl;
}


string displayWeakClassifier(const weak_classifier &wc) {
  ostringstream oss(ostringstream::out);
  oss << wc.descriptor << " descriptor." << endl;
  oss << "Theta: " << wc.theta << endl;
  oss << "Vals: " << endl << wc.vals;
  return oss.str();
}


float Dorylus::computeObjective() {
  return log_weights_.cwise().exp().sum() / (log_weights_.rows() * log_weights_.cols());
}


float Dorylus::computeUtility(const weak_classifier& wc, const VectorXf& mmt) {

  float util=0;
  int M = dd_->objs_.size();
  for(int m=0; m<M; m++) {
    if(mmt(m) == 0)
      continue;
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      util += exp(log_weights_(c, m)) * (1 - exp(-dd_->ymc_(c,m) * wc.vals(c) * mmt(m)));
    }
  }

  return util / (M * dd_->nClasses_); 
}

VectorXf Dorylus::classify(object &obj) {
  assert(nClasses_ > 0);
  VectorXf response = VectorXf::Zero(nClasses_);
  if(battery_.size() == 0)
    return response;

  map<string, vector<weak_classifier*> >::iterator bit = battery_.begin();
  for(bit = battery_.begin(); bit != battery_.end(); bit++) {
    string descriptor = bit->first;
    //cout << obj.status(false) << endl;
    //cout << descriptor << endl;
    if(obj.features.find(descriptor) == obj.features.end()) {
      cout << "Skipping " << descriptor << " descriptor, as the object has no feature of that type." << endl;
      continue;
    }
    if(find(exclude_descriptors_.begin(), exclude_descriptors_.end(), descriptor) != exclude_descriptors_.end()) {
      //cout << "Skipping " << descriptor << " descriptor, as it is on the exclude list." << endl;
      continue;
    }
    MatrixXf* f = obj.features[descriptor];
    for(int n = 0; n<f->cols(); n++) {
      vector<weak_classifier*> *act = findActivatedWCs(descriptor, f->col(n));
      for(unsigned int a = 0; a<act->size(); a++) {
	response += (*act)[a]->vals;
      }
      delete act;
    }
  }

  return response;
}

float Dorylus::classify(DorylusDataset &dd) {

  float objective=0.0;
  VectorXf response = VectorXf::Zero(nClasses_);
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    object *obj = dd.objs_[m];
    response = classify(*obj);
    assert((int)nClasses_ == response.rows());
    for(unsigned int c=0; c<nClasses_; c++) {
      objective += exp(-dd.ymc_(c, m) * response(c));
    }
  }

  objective /= dd.objs_.size() * nClasses_;
  return objective;
}
