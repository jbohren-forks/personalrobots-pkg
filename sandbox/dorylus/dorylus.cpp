#include<algorithm>
#include<dorylus.h>

using namespace std;
//using namespace NEWMAT;
USING_PART_OF_NAMESPACE_EIGEN

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
  ymc_ = NEWMAT::Matrix(nClasses_, objs.size());
  for(unsigned int m=0; m<objs.size(); m++) {
    for(unsigned int c=0; c<nClasses_; c++) {
      //If the label is 0 (BG), make all y_m^c's be -1.
      if(objs_[m]->label == (int)classes_[c] && objs_[m]->label != 0) {
	ymc_(c+1,m+1) = 1;
      }
      else
	ymc_(c+1,m+1) = -1;
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
    oss << displayObject(*objs_[i]);
  }
  return oss.str();
}

string displayObject(const object& obj) {
  map<string, MatrixXf*>::const_iterator it;
  ostringstream oss (ostringstream::out);
  oss << "Object with label " << obj.label << ":" << endl;
  for(it = obj.features.begin(); it!=obj.features.end(); it++) {
    MatrixXf* v = it->second;
    oss << it->first << " descriptor " << endl;
    oss << *v << endl;
  }
  return oss.str();
}

std::string Dorylus::status()
{
  char tmp[100];
  string st("Classifier Status: \n");
  st.append("  nWeakClassifier: ");
  sprintf(tmp, "%d \n", pwcs_.size());
  st.append(tmp);

  if(pwcs_.size() > 0) {
    map<string, vector<weak_classifier> >::iterator it;
    
    for(it=battery_.begin(); it != battery_.end(); it++) {
      sprintf(tmp, "  %s: %d\n", it->first.c_str(), it->second.size());  
      st.append(tmp);
    }
  }
  
  return st;
}



std::string DorylusDataset::status()
{
  ostringstream oss (ostringstream::out);

  oss << "DorylusDataset status: \n";
  oss << "  nClasses: " << nClasses_ << "\n";
  oss << "  nObjects: " << objs_.size() << "\n";
  oss << "  nDescriptors: " << objs_[0]->features.size() << "\n";
  oss << "  num_objs_of_class_: " << "\n";
  for(map<int, unsigned int>::iterator it=num_objs_of_class_.begin(); it != num_objs_of_class_.end(); it++) {
    oss << "    class " << (*it).first << ": " << (*it).second << " objects. \n";
  }

  map<string, MatrixXf*>::iterator fit;
  map<string, int> nPts;
  for(unsigned int m=0; m<objs_.size(); m++) {
    for(fit = objs_[m]->features.begin(); fit != objs_[m]->features.end(); fit++) {
      nPts[fit->first] += fit->second->cols();
    }
  }

  oss << "  nPts: " << endl;
  map<string, int>::iterator pit;
  for(pit = nPts.begin(); pit != nPts.end(); pit++) {
    oss << "    nPts in " << pit->first << " space: " << pit->second << "\n";
  }

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
    if(!quiet)
      cerr << "Failed to open file " << filename << endl;
    return false;
  }

  string line;
  getline(f, line);
  if(line.compare(version_string_) != 0) {
    if(!quiet)
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

      float buf;
      //What was this?  NEWMAT::Matrix v(nRows, nCols);
      pobj->features[descriptor] = new MatrixXf(nRows, nCols);
      for(int i=0; i<nCols; i++) {
	for (int j=0; j<nRows; j++) {
	  f.read((char*)&buf, sizeof(float)); 
	  (*(pobj->features[descriptor]))(j, i) = buf;
	}
      }

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

  float buf;
  map<string, vector<weak_classifier> >::iterator bit;
  for(bit = battery_.begin(); bit != battery_.end(); bit++) {
    vector<weak_classifier> &wcs = bit->second;
    for(unsigned int t=0; t<wcs.size(); t++) {
      f << "New weak classifier." << endl;
      f << "ID" << endl << wcs[t].id << endl;
      f << bit->first << endl;
      f << "Theta" << endl << wcs[t].theta << endl;
      f << "Center" << endl << wcs[t].center.rows() << endl;
      for(int i=0; i<wcs[t].center.rows(); i++) {
	buf = wcs[t].center(i,0);
	f.write((char*)&buf, sizeof(float));
      }
      f << endl;
      f << "Vals" << endl << wcs[t].vals.Nrows() << endl;
      for(int i=1; i<=wcs[t].vals.Nrows(); i++) {
	buf = wcs[t].vals(i,1);
	f.write((char*)&buf, sizeof(float));
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
    if(!quiet)
      cerr << "Failed to open file " << filename << endl;
    return false;
  }

  string line;
  getline(f, line);
  if(line.compare(version_string_) != 0) {
    if(!quiet) {
      cerr << "Log " << filename << " is of the wrong type!" << endl;
      cerr << "First line is: " << line << " instead of " << version_string_ << endl;
    }
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
	battery_[pwc->descriptor].push_back(*pwc);
	pwcs_.push_back(&battery_[pwc->descriptor].back());
	delete pwc; pwc = NULL;
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
	battery_[pwc->descriptor].push_back(*pwc);
	pwcs_.push_back(&battery_[pwc->descriptor].back());
	delete pwc; pwc = NULL;
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
      assert(line.compare("Center") == 0);
      getline(f, line);
      istringstream iss_nEle(line);
      int nElements;
      iss_nEle >> nElements;
      float buf;
      pwc->center = MatrixXf(nElements, 1);
      for(int i=0; i<nElements; i++) {
	f.read((char*)&buf, sizeof(float));
	pwc->center(i,0) = buf;
      }
      getline(f, line); //Move off the line with the data.

      getline(f, line);
      if(line.compare("Vals") != 0)
	cout << line << " is not vals!" << endl;
      assert(line.compare("Vals") == 0);
      getline(f, line);
      istringstream iss_nEleVals(line);
      iss_nEleVals >> nClasses_;
      pwc->vals = NEWMAT::Matrix(nClasses_, 1);
      for(unsigned int i=0; i<nClasses_; i++) {
	f.read((char*)&buf, sizeof(float));
	pwc->vals(i+1,1) = buf;
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

  log_weights_ = NEWMAT::Matrix(dd_->nClasses_, dd_->objs_.size());
  for(int i=1; i<=log_weights_.Nrows(); i++) {
    for(int j=1; j<=log_weights_.Ncols(); j++) {
      log_weights_(i,j) = 0;
    }
  }

 nClasses_ = dd_->nClasses_;
 classes_ = dd->classes_;
}

vector<weak_classifier*>* Dorylus::findActivatedWCs(const string &descriptor, const MatrixXf &pt) {
  vector<weak_classifier> &wcs = battery_[descriptor];
  vector<weak_classifier*> *activated = new vector<weak_classifier*>;
  activated->reserve(wcs.size());

  for(unsigned int t=0; t<wcs.size(); t++) {
    if(max_wc_ == 0 || max_wc_ > 0 && wcs[t].id <= max_wc_) {
      if(euc(pt, wcs[t].center) <= wcs[t].theta) {
	activated->push_back(&wcs[t]);
      }
    }
  }
  return activated;
}

void Dorylus::train(int nCandidates, int max_secs, int max_wcs, void (*debugHook)(weak_classifier)) {
  time_t start, end;
  time(&start);
  float obj, obj2;

  obj = computeObjective();
  cout << "Objective: " << obj << endl;
  cout << "Objective (from classify()): " << classify(*dd_) << endl;
    	  
  map<string, float> max_thetas = computeMaxThetas(*dd_);
  int wcs=0;
  while(true) {
    bool found_better = learnWC(nCandidates, max_thetas);
    if(!found_better) {
      continue;
    }
    wcs++;
    time(&end);
    if(debugHook != NULL)
      debugHook(*pwcs_.back());

    obj2 = computeObjective();
    cout << "Objective: " << obj2 << endl;

    assert(obj2 < obj);

    if(difftime(end,start) > max_secs)
      break;
    if(wcs >= max_wcs)
      break;
  }

  cout << "Done training." << endl;
}


bool Dorylus::learnWC(int nCandidates, map<string, float> max_thetas, vector<string> *desc_ignore) {
  int nThetas = 100;
  assert(dd_!=NULL);
  weak_classifier best;
  time_t start, end;
  time(&start);

  // -- Get a weights matrix that does not include the bg pts.
  map<int, unsigned int>:: iterator it = dd_->num_objs_of_class_.begin();
  int nNonBGObjs = 0;
  for(; it != dd_->num_objs_of_class_.end(); it++) {
    if(it->first == 0)
      continue;
    nNonBGObjs += it->second;
  }
  NEWMAT::Matrix non_bg_weights = NEWMAT::Matrix(nClasses_, nNonBGObjs);
  int m=0;
  for(int col=1; col<=nNonBGObjs; col++) {
    while(dd_->objs_[m]->label==0) {
      m++;
    }
    non_bg_weights.Column(col) = log_weights_.Column(m+1);
    m++;
  }
  NEWMAT::Real* pwts = non_bg_weights.Store();
  for(unsigned int i=0; i<nClasses_ * nNonBGObjs; i++) {
    pwts[i] = exp(pwts[i]);
  }

  // -- Choose wc candidates from the distribution of weights over the objects.
  vector<weak_classifier*> cand;
  for(int iCand=0; iCand<nCandidates; iCand++) {
    float dice = (float)rand() / (float)RAND_MAX * non_bg_weights.Sum(); //The weights aren't necessarily normalized.
    float w = 0.0;
    int obj_id=-1;
    int m=0;
    for(int i=0; i<non_bg_weights.Ncols(); i++, m++) {
      while(dd_->objs_[m]->label == 0)
	m++;
      w = non_bg_weights.Column(i+1).Sum();
      dice -= w;
      if(dice <= 0) {
	obj_id = m;
	break;
      }
    }
    assert(obj_id >= 0);

    // -- Get a random descriptor from the object.
    map<string, MatrixXf*> &ft = dd_->objs_[obj_id]->features;
    MatrixXf* v;
    string desc;
    while(true) {
      int desc_id = rand() % (int)ft.size();


      map<string, MatrixXf*>::iterator it = ft.begin();
      for(int i=0; i<desc_id; i++) {
	it++;
      }
      v = (*it).second;
      desc = (*it).first;

      //Make sure it's not in the ignore list.
      if(desc_ignore == NULL) {
	break;
      }
      else {
	if(find(desc_ignore->begin(), desc_ignore->end(), desc) == desc_ignore->end()) {
	  break;
	}
      }
    }

    //Get a random feature.
    int feature_id = (rand() % (int)v->cols());

    //Make the weak classifier.
    weak_classifier* wc = new weak_classifier;
    wc->descriptor = desc;
    wc->center = v->col(feature_id);
    cand.push_back(wc);
  }
  cout << "Added " << nCandidates << " candidate wcs" << endl;


  // -- For all candidates, try several thetas and get their utilities.
//   NEWMAT::Matrix best_weights, *pweights=NULL;
//   NEWMAT::Matrix **ppweights = &pweights;
//  float objective = computeObjective();
  float max_util = 0.0;
  NEWMAT::Matrix best_mmt;
  bool found_better = false;
  for(unsigned int i=0; i<cand.size(); i++) {

    // -- Get the distances from this candidate to all other points.
    vector<NEWMAT::Matrix> dists;
    int nPts = 0;
    dists.reserve(dd_->objs_.size());
    for(unsigned int j=0; j<dd_->objs_.size(); j++) {
      MatrixXf* f = dd_->objs_[j]->features[cand[i]->descriptor];
      dists.push_back(NEWMAT::Matrix(1, f->cols()));
      nPts += f->cols();
    }

    int j=0,k=0;
    MatrixXf *f;

    for(j=0; j<(int)dd_->objs_.size(); j++) {
      f = dd_->objs_[j]->features[cand[i]->descriptor];
      for(k=1; k<=f->cols(); k++) {
	dists[j](1, k) = euc(f->col(k-1), cand[i]->center);
      }
    }


    cand[i]->theta = 0;
    for(int iTheta=0; iTheta<nThetas; iTheta++) {
      float MAX_THETA = max_thetas[cand[i]->descriptor];

      // -- Rejection sampling to take small thetas more often.
      while(true) {
	cand[i]->theta = ((float)rand() / (float)RAND_MAX) * MAX_THETA;
	float probability = -2 / (MAX_THETA * MAX_THETA) * cand[i]->theta + 2 / MAX_THETA;
	float random = (float)rand() / (float)RAND_MAX;
	if(random < probability)
	  break;
      }

      //Get the M_m^t, number of points in the hypersphere for each object.
      NEWMAT::Matrix mmt(1,dd_->objs_.size()); mmt=0.0;
      for(unsigned int j=0; j<dists.size(); j++) {
	for(int k=1; k<=dists[j].Ncols(); k++) {
	  if(dists[j](1,k) < cand[i]->theta)
	    mmt(1, j+1)++;
	}
      }

//       // -- AdaBoost: Use Newton's method to solve for the a_t^c's.
//       cand[i]->vals = NEWMAT::Matrix(dd_->nClasses_, 1); cand[i]->vals = 0.0;
//       for(unsigned int c=0; c<dd_->nClasses_; c++) {
// 	//TODO: Check if the function is unbounded below.

// 	Function fcn(this, mmt, c);
// 	Gradient grad(this, mmt, c);
// 	Hessian hes(this, mmt, c);

// 	cand[i]->vals(c+1,1) = newtonSingle(fcn, grad, hes, 1e-6);
//       }

      // -- GentleBoost: take a single newton step to set the a_t^c's.
      cand[i]->vals = NEWMAT::Matrix(dd_->nClasses_, 1); cand[i]->vals = 0.0;

      int idx=0;
      int M = dd_->objs_.size();
      NEWMAT::Real* pmmt = mmt.Store();
      NEWMAT::Real* pymc = dd_->ymc_.Store();
      NEWMAT::Real* plog_weights_ = log_weights_.Store();
      NEWMAT::Matrix numerators(nClasses_,1);
      NEWMAT::Matrix denominators(nClasses_,1);
      numerators = 0.0;
      denominators = 0.0;
      NEWMAT::Real* pnums = numerators.Store();
      NEWMAT::Real* pdens = denominators.Store();
      for(int m=0; m<M; m++) {
	if(pmmt[m] == 0)
	  continue;
	for(unsigned int c=0; c<dd_->nClasses_; c++) {
	  idx = m + M*c;
	  pnums[c] += exp(plog_weights_[idx]) * pymc[idx] * pmmt[m];
	  pdens[c] += exp(plog_weights_[idx]) * pmmt[m] * pmmt[m];
	}

// 	// -- Non-negative responses test.
// 	if(cand[i]->vals(c+1,1) < 0) {
// 	  cand[i]->vals(c+1,1) = 0;
// 	}

      }
      for(unsigned int c=0; c<dd_->nClasses_; c++) {
	if(pdens[c]==0)
	  cand[i]->vals(c+1,1) = 0;
	else
	  cand[i]->vals(c+1,1) = pnums[c] / pdens[c];
      }


      //Get the utility.
  //     float new_objective = computeNewObjective(cand[i], mmt, ppweights);
//       float util = objective - new_objective;
      float util = computeUtility(*cand[i], mmt);
      //      cout << "util " << util << " " << util2 << endl;

      if(util > max_util) {
	found_better = true;
	max_util = util;
	best = *cand[i];
	//best_weights = **ppweights;
	best_mmt = mmt;
      }
      //delete *ppweights; *ppweights = NULL;
    }
    cout << "."; cout.flush();
  }
  cout << endl;

  if(!found_better) {
    cout << "Did not find a weak classifier that improves the classification!" << endl;
    return false;
  }

  cout << "Found wc with utility " << max_util << endl;

  // -- Add the best to the strong classifier.
  best.id = pwcs_.size()+1;
  battery_[best.descriptor].push_back(best);
  pwcs_.push_back(&battery_[best.descriptor].back());

  time(&end);
  cout << "Took " << difftime(end,start) << " seconds to try " << nCandidates << " wcs with " << nThetas << " thetas each." << endl;
  cout << displayWeakClassifier(best) << endl;
  int nObjs_encompassed = 0;
  for(int i=1; i<=best_mmt.Ncols(); i++) {
    if(best_mmt(1,i) != 0)
      nObjs_encompassed++;
  }
  cout << "WC encompasses at least one point from " << nObjs_encompassed << " out of " << best_mmt.Ncols() << " objects." << endl;

  // -- Compute the new log weights.
  for(unsigned int m=0; m<dd_->objs_.size(); m++) {
    if(best_mmt(1, m+1) == 0)
      continue;
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      log_weights_(c+1, m+1) += -dd_->ymc_(c+1, m+1) * best.vals(c+1,1) * best_mmt(1, m+1);
    }
  }
  //  delete *ppweights; *ppweights = NULL;
  return true;
}

// Choose MAX_THETA by fitting a multidimensional gaussian using MLE.
map<string, float> Dorylus::computeMaxThetas(const DorylusDataset &dd) {
  map<string, float> max_thetas;
  map<string, float> variances;
  map<string, VectorXf> means;
  map<string, float> nPts;

  // -- Get the means and the nPts for each descriptor space.
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    map<string, MatrixXf*>::const_iterator fit = dd.objs_[m]->features.begin();
    for(; fit!=dd.objs_[m]->features.end(); fit++) {
      string const& descr = fit->first;
      MatrixXf* f = fit->second;
      if(means.find(descr) == means.end()) {
	means[descr] = VectorXf::Zero(f->rows());
      }
      if(nPts.find(descr) == nPts.end())
	nPts[descr] = 0;

      for(int k=0; k<f->cols(); k++) {
	means[descr] += f->col(k);
	nPts[descr]++;
      }
    }
  }

  map<string, VectorXf>::iterator mit;
  for(mit = means.begin(); mit != means.end(); mit++) {
    mit->second = mit->second / nPts[mit->first];
//     NEWMAT::Matrix div(1,1); div = 1 / nPts[mit->first];
//     mit->second = KP(mit->second, div);
  }

  // -- Get the variances for each descriptor space.
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    map<string, MatrixXf*>::const_iterator fit = dd.objs_[m]->features.begin();
    for(; fit!=dd.objs_[m]->features.end(); fit++) {
      string const& descr = fit->first;
      if(variances.find(descr) == variances.end())
	variances[descr] = 0;

      MatrixXf* f = fit->second;
      for(int k=0; k<f->cols(); k++) {
	VectorXf zeroed = f->col(k) - means[descr];
	//cout << "zeroed = " << zeroed<<endl;
	variances[descr] += zeroed.dot(zeroed);
      }
    }
  }

  map<string, float>::iterator vit;
  for(vit = variances.begin(); vit != variances.end(); vit++) {
    vit->second = vit->second / (nPts[vit->first]); // * means[vit->first].Nrows());  Use E[x^T x] for zero-mean data.
    max_thetas[vit->first] = 2*sqrt(vit->second); //2 stdev.
  }

  // -- Display some statistics.
  for(vit = variances.begin(); vit != variances.end(); vit++) {
    cout << vit->first << " variance: " << vit->second << ", nPts: " << nPts[vit->first] << ", max_theta: " << max_thetas[vit->first] << endl;
  }

  return max_thetas;
}



string displayWeakClassifier(const weak_classifier &wc) {
  ostringstream oss(ostringstream::out);
  oss << wc.descriptor << " descriptor." << endl;
  oss << "Theta: " << wc.theta << endl;
  oss << "Vals: " << endl << wc.vals;
  return oss.str();
}


float Dorylus::computeObjective() {
  float obj=0;
  NEWMAT::Real* plog_weights = log_weights_.Store();
  for(int i=0; i<log_weights_.Nrows() * log_weights_.Ncols(); i++) {
    obj += exp(plog_weights[i]);
  }
  obj /= log_weights_.Nrows() * log_weights_.Ncols();
  return obj;
}

//mmt is a M_m^t specific to this weak classifier and this dataset.
NEWMAT::Matrix Dorylus::computeDatasetActivations(const weak_classifier& wc, const NEWMAT::Matrix& mmt) {
  NEWMAT::Matrix act(dd_->nClasses_, dd_->objs_.size()); act = 0.0;

  for(unsigned int m=0; m<dd_->objs_.size(); m++) {

    //This can become an outer product.
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      act(c+1, m+1) += wc.vals(c+1,1) * mmt(1, m+1);
    }
  }

  NEWMAT::Matrix act2(dd_->nClasses_, dd_->objs_.size());
  act2 = wc.vals * mmt;

  for(unsigned int m=0; m<dd_->objs_.size(); m++) {
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      if(act2(c+1, m+1) != act(c+1, m+1)) {
	cout << act2(c+1, m+1) << " " <<  act(c+1, m+1) << endl;
	cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
  }


  return act2;
}


float Dorylus::computeUtility(const weak_classifier& wc, const NEWMAT::Matrix& mmt) {
//   float util=0;
//   for(unsigned int m=0; m<dd_->objs_.size(); m++) {
//     if(mmt(1, m+1) == 0)
//       continue;
//     for(unsigned int c=0; c<dd_->nClasses_; c++) {
//       util += exp(log_weights_(c+1,m+1)) * (1 - exp(-dd_->ymc_(c+1, m+1) * wc.vals(c+1,1) * mmt(1,m+1)));
//     }
//   }

  NEWMAT::Real* pvals = wc.vals.Store();
  NEWMAT::Real* pmmt = mmt.Store();
  NEWMAT::Real* pymc = dd_->ymc_.Store();
  NEWMAT::Real* plog_weights_ = log_weights_.Store();
  float util=0;
  int idx;
  int M = dd_->objs_.size();
  for(int m=0; m<M; m++) {
    if(pmmt[m] == 0)
      continue;
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      idx = m + c*M;
      util += exp(plog_weights_[idx]) * (1 - exp(-pymc[idx] * pvals[c] * pmmt[m]));
    }
  }

  return util / (M * dd_->nClasses_);
}


// float Dorylus::computeNewObjective(const weak_classifier& wc, const NEWMAT::Matrix& mmt, NEWMAT::Matrix** new_weights) {
//   NEWMAT::Matrix weights = weights_;
//   NEWMAT::Matrix log_weights = log_weights_;
//   //NEWMAT::Matrix act = computeDatasetActivations(wc, mmt);
//   NEWMAT::Matrix act;
//   act = wc.vals * mmt;


//   // -- Compute the new weights.
//   float new_weight;
//   for(unsigned int m=0; m<dd_->objs_.size(); m++) {
//     if(mmt(1, m+1) == 0)
//       continue;

//     for(unsigned int c=0; c<dd_->nClasses_; c++) {
//       new_weight = weights(c+1, m+1) * exp(-dd_->ymc_(c+1, m+1) * act(c+1,m+1));

//       if(new_weight == 0) {
// 	new_weight = FLT_MIN;
//       }

//       weights(c+1, m+1) = new_weight;
//     }
//   }

//   // -- Compute the new log weights.
//   for(unsigned int m=0; m<dd_->objs_.size(); m++) {
//     if(mmt(1, m+1) == 0)
//       continue;
//     for(unsigned int c=0; c<dd_->nClasses_; c++) {
//       log_weights(c+1, m+1) += -dd_->ymc_(c+1, m+1) * act(c+1,m+1);
//     }
//   }

//   if(new_weights != NULL) {
//     *new_weights = new NEWMAT::Matrix;
//     **new_weights = weights;
//   }

//   return weights.Sum();
// }

NEWMAT::Matrix Dorylus::classify(object &obj, NEWMAT::Matrix **confidence) {
  NEWMAT::Matrix response(nClasses_, 1); response = 0.0;
  if(battery_.size() == 0)
    return response;

  map<string, vector<weak_classifier> >::iterator bit = battery_.begin();
  for(bit = battery_.begin(); bit != battery_.end(); bit++) {
    string descriptor = bit->first;
    if(obj.features.find(descriptor) == obj.features.end()) {
      //cout << "Skipping " << descriptor << " descriptor, as the object has no feature of that type." << endl;
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
  NEWMAT::Matrix response(nClasses_, 1);
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    object *obj = dd.objs_[m];
    response = classify(*obj);
    assert((int)nClasses_ == response.Nrows());
    for(unsigned int c=0; c<nClasses_; c++) {
      objective += exp(-dd.ymc_(c+1, m+1 ) * response(c+1,1));
    }
  }

  objective /= dd.objs_.size() * nClasses_;
  return objective;
}
