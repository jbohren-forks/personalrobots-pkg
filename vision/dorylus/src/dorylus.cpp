#include<omp.h>
#include<dorylus.h>

using namespace std;
#define Matrix NEWMAT::Matrix //TODO: Fix this dirty hack.
#define Real NEWMAT::Real

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
    

void DorylusDataset::setObjs(const vector<object>& objs) 
{
  //num_class_objs_ contains the number of bg objs.  classes_ does NOT, nor does nClasses_
  objs_ = objs;
  num_class_objs_.clear();
  classes_.clear();

  // -- Get num_class_objs_.
  for(unsigned int m=0; m<objs.size(); m++) {
    // Don't count label=0 as a class!
    if(num_class_objs_.find(objs[m].label) == num_class_objs_.end() && objs[m].label != 0)
      classes_.push_back(objs[m].label);
    num_class_objs_[objs[m].label] = 0;
  }
  for(unsigned int m=0; m<objs.size(); m++) {
    num_class_objs_[objs[m].label]++;
  }
  nClasses_ = classes_.size();

  
  // -- Reconstruct ymc_
  ymc_ = Matrix(nClasses_, objs.size());
  for(unsigned int m=0; m<objs.size(); m++) {
    for(unsigned int c=0; c<nClasses_; c++) {
      //If the label is 0 (BG), make all y_m^c's be -1.
      if(objs_[m].label == (int)classes_[c] && objs_[m].label != 0) {
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

string DorylusDataset::displayFeatures() {
  ostringstream oss (ostringstream::out);
  map<string, Matrix>::iterator it;
  for(unsigned int i=0; i<objs_.size(); i++) {
    object& obj = objs_[i];
    oss << "Object " << i << " " << endl;
    for(it = obj.features.begin(); it!=obj.features.end(); it++) {
      Matrix& v = it->second;
      oss << it->first << " descriptor " << endl;
      oss << v;
    }
  }
  oss << "ymc_: " << endl << ymc_ << endl;
  return oss.str();
}
  

std::string Dorylus::status()
{
  ostringstream oss (ostringstream::out);
  oss << version_string_ << " status: \n";
  oss << "  nWeakClassifier: " << pwcs_.size() << endl;
  map<string, vector<weak_classifier> >::iterator it;
  oss << "  weak classifiers: \n";
  for(it=battery_.begin(); it != battery_.end(); it++) {
    oss << "    " << it->first << ": " << it->second.size() << " weak classifiers." << endl;
  }
  return oss.str();
}



std::string DorylusDataset::status()
{
  ostringstream oss (ostringstream::out);
  
  oss << "DorylusDataset status: \n";
  oss << "  nClasses: " << nClasses_ << "\n";
  oss << "  nObjects: " << objs_.size() << "\n";
  oss << "  nDescriptors: " << objs_[0].features.size() << "\n";
  oss << "  num_class_objs_: " << "\n";
  for(map<int, unsigned int>::iterator it=num_class_objs_.begin(); it != num_class_objs_.end(); it++) {
    oss << "    class " << (*it).first << ": " << (*it).second << " objects. \n";
  }

  map<string, Matrix>::iterator fit;
  map<string, int> nPts;
  for(unsigned int m=0; m<objs_.size(); m++) {
    for(fit = objs_[m].features.begin(); fit != objs_[m].features.end(); fit++) {
      nPts[fit->first] += fit->second.Ncols();
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
    object& obj = objs_[i];
    f << "New object." << endl;
    f << obj.label << endl;
    map<string, Matrix>::iterator it;
    for(it = obj.features.begin(); it!=obj.features.end(); it++) {
      Matrix& v = it->second;
      f << "New descriptor." << endl;
      f << it->first << endl;
      f << v.Nrows() << endl;
      f << v.Ncols() << endl;
      for(int k=1; k<=v.Ncols(); k++) {
	for(int m=1; m<=v.Nrows(); m++) {
	  buf = v(m,k);
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

  vector<object> objs;
  int nRows, nCols;
  string descriptor;
  object *pobj = NULL;
  while(true) {

    getline(f, line);

    if(line.size() == 0) {
      if(pobj) {
	objs.push_back(*pobj);
	delete pobj; pobj = NULL;
      }	
      else {
	cerr << "no object??" << endl;
      }
      break;
    }

    else if(line.compare(string("New object.")) == 0) {
      if(pobj) {
	objs.push_back(*pobj);
	delete pobj; pobj = NULL;
      }	
      pobj = new object;
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
      Matrix v(nRows, nCols);
      pobj->features[descriptor] = Matrix(nRows, nCols);
      for(int i=0; i<nCols; i++) {
	for (int j=0; j<nRows; j++) {
	  f.read((char*)&buf, sizeof(float));
	  pobj->features[descriptor](j+1, i+1) = buf;
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


bool DorylusDataset::testSave()
{
  cout << "Running save test." << endl;
  DorylusDataset dd;

  map<string, Matrix> f;
  Matrix spin(3,2);
  spin(1,1) = 1;
  spin(2,1) = 2;
  spin(3,1) = 3;
  spin(1,2) = 4;
  spin(2,2) = 5;
  spin(3,2) = 6;
  Matrix sift(3,2); sift = 2.2;
  sift(1,1) = 101;
  sift(2,2) = 102;
  f["spinimg"] = spin;
  f["sift"] = sift;

  object obj;
  obj.label = 0;
  obj.features = f;
  vector<object> objs;
  objs.push_back(obj);
  obj.label = 1;
  objs.push_back(obj);
  dd.setObjs(objs);

  cout << dd.status() << endl;
  dd.save(string("test.dd"));
  
  DorylusDataset dd2;
  dd2.load(string("test.dd"));
  dd2.save(string("test2.dd"));
  cout << dd2.status() << endl;

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
      f << "Center" << endl << wcs[t].center.Nrows() << endl;
      for(int i=1; i<=wcs[t].center.Nrows(); i++) {
	buf = wcs[t].center(i,1);
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
      pwc->center = Matrix(nElements, 1);
      for(int i=0; i<nElements; i++) {
	f.read((char*)&buf, sizeof(float));
	pwc->center(i+1,1) = buf;
      }
      getline(f, line); //Move off the line with the data.

      getline(f, line);
      if(line.compare("Vals") != 0)
	cout << line << " is not vals!" << endl;
      assert(line.compare("Vals") == 0);
      getline(f, line);
      istringstream iss_nEleVals(line);
      iss_nEleVals >> nClasses_;
      pwc->vals = Matrix(nClasses_, 1);
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

    
void Dorylus::loadDataset(DorylusDataset *dd) {
  dd_ = dd;
//   weights_ = Matrix(dd_->nClasses_, dd_->objs_.size());
//   for(int i=1; i<=weights_.Nrows(); i++) {
//     for(int j=1; j<=weights_.Ncols(); j++) {
//       weights_(i,j) = 1;
//     }
//   }
  log_weights_ = Matrix(dd_->nClasses_, dd_->objs_.size());
  for(int i=1; i<=log_weights_.Nrows(); i++) {
    for(int j=1; j<=log_weights_.Ncols(); j++) {
      log_weights_(i,j) = 0;
    }
  }
 
  //normalizeWeights();
 nClasses_ = dd_->nClasses_;
 classes_ = dd->classes_;
}


// void Dorylus::normalizeWeights() {
//   Matrix n(1,1); n = (1 / weights_.Sum());
//   weights_ = KP(weights_, n);

//   // -- Make sure no weights are zero.
//   for(int i=1; i<weights_.Nrows(); i++) {
//     for(int j=1; j<weights_.Ncols(); j++) {
//       if(weights_(i,j) == 0)
// 	weights_(i,j) = FLT_MIN;
//     }
//   }
//}

vector<weak_classifier*> Dorylus::findActivatedWCs(const string &descriptor, const Matrix &pt) {
  vector<weak_classifier> &wcs = battery_[descriptor];
  vector<weak_classifier*> activated;

  for(unsigned int t=0; t<wcs.size(); t++) {
    if(euc(pt, wcs[t].center) <= wcs[t].theta)
      activated.push_back(&wcs[t]);
  }
  return activated;
}

bool Dorylus::learnWC(int nCandidates, map<string, float> max_thetas, vector<string> *desc_ignore) {
  int nThetas = 100;
  assert(dd_!=0);
  weak_classifier best;
  time_t start, end;
  time(&start);

  // -- Get a weights matrix that does not include the bg pts.
  map<int, unsigned int>:: iterator it = dd_->num_class_objs_.begin();
  int nNonBGObjs = 0;
  for(; it != dd_->num_class_objs_.end(); it++) {
    if(it->first == 0)
      continue;
    nNonBGObjs += it->second;
  }
  Matrix non_bg_weights = Matrix(nClasses_, nNonBGObjs);
  int m=0;
  for(int col=1; col<=nNonBGObjs; col++) {
    while(dd_->objs_[m].label==0) {
      m++;
    }
    non_bg_weights.Column(col) = log_weights_.Column(m+1);
    m++;
  }
  Real* pwts = non_bg_weights.Store();
  for(unsigned int i=0; i<nClasses_ * nNonBGObjs; i++) {
    pwts[i] = exp(pwts[i]);
  }

  // -- Choose wc candidates from the distribution of weights over the objects.
  vector<weak_classifier> cand;
  for(int iCand=0; iCand<nCandidates; iCand++) {
    float dice = (float)rand() / (float)RAND_MAX * non_bg_weights.Sum(); //The weights aren't necessarily normalized.
    float w = 0.0;
    int obj_id=-1;
    int m=0;
    for(int i=0; i<non_bg_weights.Ncols(); i++, m++) {
      while(dd_->objs_[m].label == 0)
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
    map<string, Matrix> &ft = dd_->objs_[obj_id].features;
    Matrix v;
    string desc;
    while(true) {
      int desc_id = rand() % (int)ft.size();
      

      map<string, Matrix>::iterator it = ft.begin();
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
    int feature_id = (rand() % (int)v.Ncols()) + 1;

    //Make the weak classifier.
    weak_classifier wc;
    wc.descriptor = desc;
    wc.center = v.Column(feature_id);
    cand.push_back(wc);
  }
  cout << "Added " << nCandidates << " candidate wcs" << endl;
    

  // -- For all candidates, try several thetas and get their utilities.
//   Matrix best_weights, *pweights=NULL;
//   Matrix **ppweights = &pweights;
//  float objective = computeObjective();
  float max_util = 0.0;
  Matrix best_mmt;
  bool found_better = false;
  for(unsigned int i=0; i<cand.size(); i++) {
    
    // -- Get the distances from this candidate to all other points.
    vector<Matrix> dists;
    int nPts = 0;
    dists.reserve(dd_->objs_.size());
    for(unsigned int j=0; j<dd_->objs_.size(); j++) {
      Matrix &f = dd_->objs_[j].features[cand[i].descriptor];
      dists.push_back(Matrix(1, f.Ncols()));  
      nPts += f.Ncols();
    }
    //int chunk = nPts / 2;
    int j=0,k=0;
    Matrix *f;

    //#pragma omp parallel default(shared) private(j,k,f)
    {

      //#pragma omp for schedule(dynamic,chunk)
      for(j=0; j<(int)dd_->objs_.size(); j++) {
	f = &dd_->objs_[j].features[cand[i].descriptor];
	for(k=1; k<=f->Ncols(); k++) {
	  dists[j](1, k) = euc(f->Column(k), cand[i].center);
	}
      }

    }

    cand[i].theta = 0;
    for(int iTheta=0; iTheta<nThetas; iTheta++) {
      float MAX_THETA = max_thetas[cand[i].descriptor];

      // -- Rejection sampling to take small thetas more often.
      while(true) {
	cand[i].theta = ((float)rand() / (float)RAND_MAX) * MAX_THETA;
	float probability = -2 / (MAX_THETA * MAX_THETA) * cand[i].theta + 2 / MAX_THETA;
	float random = (float)rand() / (float)RAND_MAX;
	if(random < probability)
	  break;
      }

      //Get the M_m^t, number of points in the hypersphere for each object.
      Matrix mmt(1,dd_->objs_.size()); mmt=0.0;
      for(unsigned int j=0; j<dists.size(); j++) {
	for(int k=1; k<=dists[j].Ncols(); k++) {
	  if(dists[j](1,k) < cand[i].theta)
	    mmt(1, j+1)++;
	}
      }

//       // -- AdaBoost: Use Newton's method to solve for the a_t^c's.
//       cand[i].vals = Matrix(dd_->nClasses_, 1); cand[i].vals = 0.0;
//       for(unsigned int c=0; c<dd_->nClasses_; c++) {
// 	//TODO: Check if the function is unbounded below.

// 	Function fcn(this, mmt, c);
// 	Gradient grad(this, mmt, c);
// 	Hessian hes(this, mmt, c);

// 	cand[i].vals(c+1,1) = newtonSingle(fcn, grad, hes, 1e-6);
//       }

      // -- GentleBoost: take a single newton step to set the a_t^c's.
      cand[i].vals = Matrix(dd_->nClasses_, 1); cand[i].vals = 0.0;

      int idx=0;
      int M = dd_->objs_.size();
      Real* pmmt = mmt.Store();
      Real* pymc = dd_->ymc_.Store();
      Real* plog_weights_ = log_weights_.Store();
      Matrix numerators(nClasses_,1);
      Matrix denominators(nClasses_,1);
      numerators = 0.0;
      denominators = 0.0;
      Real* pnums = numerators.Store();
      Real* pdens = denominators.Store();
      for(int m=0; m<M; m++) {
	if(pmmt[m] == 0)
	  continue;
	for(unsigned int c=0; c<dd_->nClasses_; c++) {
	  idx = m + M*c;
	  pnums[c] += exp(plog_weights_[idx]) * pymc[idx] * pmmt[m];
	  pdens[c] += exp(plog_weights_[idx]) * pmmt[m] * pmmt[m];
	}

// 	// -- Non-negative responses test.
// 	if(cand[i].vals(c+1,1) < 0) {
// 	  cand[i].vals(c+1,1) = 0;
// 	}

      }
      for(unsigned int c=0; c<dd_->nClasses_; c++) {
	if(pdens[c]==0)
	  cand[i].vals(c+1,1) = 0;
	else
	  cand[i].vals(c+1,1) = pnums[c] / pdens[c];
      }


      //Get the utility.
  //     float new_objective = computeNewObjective(cand[i], mmt, ppweights);
//       float util = objective - new_objective;
      float util = computeUtility(cand[i], mmt);
      //      cout << "util " << util << " " << util2 << endl;

      if(util > max_util) {
	found_better = true;
	max_util = util;
	best = cand[i];
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
  map<string, Matrix> means;
  map<string, float> nPts;

  // -- Get the means and the nPts for each descriptor space.
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    map<string, Matrix>::const_iterator fit = dd.objs_[m].features.begin();
    for(; fit!=dd.objs_[m].features.end(); fit++) {
      string const& descr = fit->first;
      Matrix const& f = fit->second;
      if(means.find(descr) == means.end()) {
	means[descr] = Matrix(f.Nrows(), 1);
	means[descr] = 0.0;
      }
      if(nPts.find(descr) == nPts.end())
	nPts[descr] = 0;

      for(int k=1; k<=f.Ncols(); k++) {
	means[descr] += f.Column(k);
	nPts[descr]++;
      }
    }
  }

  map<string, Matrix>::iterator mit;
  for(mit = means.begin(); mit != means.end(); mit++) {
    Matrix div(1,1); div = 1 / nPts[mit->first];
    mit->second = KP(mit->second, div);
  }

  // -- Get the variances for each descriptor space.
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    map<string, Matrix>::const_iterator fit = dd.objs_[m].features.begin();
    for(; fit!=dd.objs_[m].features.end(); fit++) {
      string const& descr = fit->first;
      if(variances.find(descr) == variances.end())
	variances[descr] = 0;

      Matrix const& f = fit->second;
      for(int k=1; k<=f.Ncols(); k++) {
	Matrix zeroed = f.Column(k) - means[descr];
	variances[descr] += DotProduct(zeroed, zeroed);
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
  Real* plog_weights = log_weights_.Store();
  for(int i=0; i<log_weights_.Nrows() * log_weights_.Ncols(); i++) {
    obj += exp(plog_weights[i]);
  }
  obj /= log_weights_.Nrows() * log_weights_.Ncols();
  return obj;
}

//mmt is a M_m^t specific to this weak classifier and this dataset.
Matrix Dorylus::computeDatasetActivations(const weak_classifier& wc, const Matrix& mmt) {
  Matrix act(dd_->nClasses_, dd_->objs_.size()); act = 0.0;
  
  for(unsigned int m=0; m<dd_->objs_.size(); m++) {

    //This can become an outer product.
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      act(c+1, m+1) += wc.vals(c+1,1) * mmt(1, m+1);
    }
  }

  Matrix act2(dd_->nClasses_, dd_->objs_.size());
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


float Dorylus::computeUtility(const weak_classifier& wc, const Matrix& mmt) {
//   float util=0;
//   for(unsigned int m=0; m<dd_->objs_.size(); m++) {
//     if(mmt(1, m+1) == 0) 
//       continue;
//     for(unsigned int c=0; c<dd_->nClasses_; c++) {
//       util += exp(log_weights_(c+1,m+1)) * (1 - exp(-dd_->ymc_(c+1, m+1) * wc.vals(c+1,1) * mmt(1,m+1)));
//     }
//   }
  
  Real* pvals = wc.vals.Store();
  Real* pmmt = mmt.Store();
  Real* pymc = dd_->ymc_.Store();
  Real* plog_weights_ = log_weights_.Store();
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

  return util;
}


// float Dorylus::computeNewObjective(const weak_classifier& wc, const Matrix& mmt, Matrix** new_weights) {
//   Matrix weights = weights_;
//   Matrix log_weights = log_weights_;
//   //Matrix act = computeDatasetActivations(wc, mmt);
//   Matrix act;
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
//     *new_weights = new Matrix;
//     **new_weights = weights;
//   }

//   return weights.Sum();
// }

Matrix Dorylus::classify(object &obj, Matrix **confidence) {
  Matrix response(nClasses_, 1); response = 0.0;
  if(battery_.size() == 0)
    return response;

  map<string, vector<weak_classifier> >::iterator bit = battery_.begin();
  for(bit = battery_.begin(); bit != battery_.end(); bit++) {
    string descriptor = bit->first;
    if(obj.features.find(descriptor) == obj.features.end()) {
      cout << "Skipping " << descriptor << " descriptor." << endl;
      continue;
    }
    Matrix &f = obj.features[descriptor];
    for(int n = 1; n<=f.Ncols(); n++) {
      vector<weak_classifier*> act = findActivatedWCs(descriptor, f.Column(n));
      for(unsigned int a = 0; a<act.size(); a++) {
	response += act[a]->vals;
      }
    }
  }
  
  return response;
}

float Dorylus::classify(DorylusDataset &dd) {
  
  float objective=0.0;
  Matrix response(nClasses_, 1);
  for(unsigned int m=0; m<dd.objs_.size(); m++) {
    object &obj = dd.objs_[m];
    response = classify(obj);
    assert((int)nClasses_ == response.Nrows());
    for(unsigned int c=0; c<nClasses_; c++) {
      objective += exp(-dd.ymc_(c+1, m+1 ) * response(c+1,1));
    }
  }

  objective /= dd.objs_.size() * nClasses_;
  return objective;
}
