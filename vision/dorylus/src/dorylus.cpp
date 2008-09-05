
#include<dorylus.h>

using namespace std;
#define Matrix NEWMAT::Matrix
//using namespace NEWMAT;

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
  objs_ = objs;

  num_class_objs_.clear();
  classes_.clear();

  // -- Get num_class_objs_.
  for(unsigned int m=0; m<objs.size(); m++) {
    if(num_class_objs_.find(objs[m].label) == num_class_objs_.end())
      classes_.push_back(objs[m].label);
    num_class_objs_[objs[m].label] = 0;
  }
  for(unsigned int m=0; m<objs.size(); m++) {
    num_class_objs_[objs[m].label]++;
  }
  nClasses_ = num_class_objs_.size();
  
  // -- Reconstruct ymc_
  ymc_ = Matrix(nClasses_, objs.size());
  for(unsigned int m=0; m<objs.size(); m++) {
    for(unsigned int c=0; c<nClasses_; c++) {
      if(objs_[m].label == (int)classes_[c]) {
	ymc_(c+1,m+1) = 1;
      }
      else
	ymc_(c+1,m+1) = -1;
    }
  }
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
  //oss << "ymc_: " << endl << ymc_ << endl;
  //  oss << "object data" << endl << displayFeatures() << endl;
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

bool DorylusDataset::load(string filename) 
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

bool Dorylus::save(string filename) 
{
  ofstream f;
  f.open(filename.c_str());
  if(f.fail()) {
    cerr << "Failed to open file " << filename << endl;
    return false;
  }

  f << version_string_ << endl;
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
  f.close();
  return true; 
}


bool Dorylus::load(string filename) 
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

  battery_.clear();
  string descriptor;
  weak_classifier *pwc = NULL;
  while(true) {

    getline(f, line);

    if(line.size() == 0) {
      if(pwc) {
	battery_[pwc->descriptor].push_back(*pwc);
	cout << "Stored new wc." << endl;
	cout << displayWeakClassifier(*pwc);
	delete pwc; pwc = NULL;
      }	
      else {
	cerr << "no wc??" << endl;
      }
      break;
    }

    else if(line.compare(string("New weak classifier.")) == 0) {	
      if(pwc) {
	battery_[pwc->descriptor].push_back(*pwc);
	cout << "Stored new wc." << endl;
	cout << displayWeakClassifier(*pwc);
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
  weights_ = Matrix(dd_->nClasses_, dd_->objs_.size());


  for(int i=1; i<=weights_.Nrows(); i++) {
    for(int j=1; j<=weights_.Ncols(); j++) {
      weights_(i,j) = 1;
    }
  }
 
 normalizeWeights();
 nClasses_ = dd_->nClasses_;
}


void Dorylus::normalizeWeights() {
  Matrix n(1,1); n = (1 / weights_.Sum());
  weights_ = KP(weights_, n);

  // -- Make sure no weights are zero.
  for(int i=1; i<weights_.Nrows(); i++) {
    for(int j=1; j<weights_.Ncols(); j++) {
      if(weights_(i,j) == 0)
	weights_(i,j) = FLT_MIN;
    }
  }
}
	


vector<weak_classifier*> Dorylus::findActivatedWCs(const string &descriptor, const Matrix &pt) {
  vector<weak_classifier> &wcs = battery_[descriptor];
  vector<weak_classifier*> activated;
  for(unsigned int t=0; t<wcs.size(); t++) {
    if(euc(pt, wcs[t].center) <= wcs[t].theta)
      activated.push_back(&wcs[t]);
  }
  return activated;
}

void Dorylus::train(int nCandidates, int max_secs, int max_wcs) {
  time_t start, end;
  time(&start);
  
  cout << "Objective: " << computeObjective() << endl;
  cout << "Objective (from classify()): " << classify(*dd_) << endl;
  int wcs=0;
  while(true) {
    learnWC(nCandidates);
    wcs++;
    time(&end);
    cout << "Objective: " << computeObjective() << endl;
    cout << "Objective (from classify()): " << classify(*dd_) << endl;
    cout << "Difference: " << computeObjective() - classify(*dd_) << endl;

    if(difftime(end,start) > max_secs)
      break;
    if(wcs >= max_wcs)
      break;
  }

  cout << "Done training." << endl;
}

void Dorylus::learnWC(int nCandidates) {
  int nThetas = 500;
  assert(dd_!=0);
  weak_classifier best;
  time_t start, end;
  time(&start);

  // -- Choose wc candidates from the distribution of weights over the objects.

  vector<weak_classifier> cand;
  for(int iCand=0; iCand<nCandidates; iCand++) {
    //cout << "Constructing candidate " << iCand << endl;
    float dice = (float)rand() / (float)RAND_MAX * weights_.Sum(); //The weights aren't necessarily normalized.
    //cout << "dice " << dice << endl;
    Matrix &ws = weights_;
    //cout << ws << endl;
    float w = 0.0;
    int obj_id=-1;
    for(int i=0; i<ws.Ncols(); i++) {
      w = ws.Column(i+1).Sum();
      dice -= w;
      //cout << "dice " << dice << endl;
      if(dice < 0) {
	obj_id = i;
	break;
      }
    }
    assert(obj_id >= 0);
    //cout << "Picking from obj " << obj_id << endl;

    //Get a random descriptor.
    map<string, Matrix> &ft = dd_->objs_[obj_id].features;
    //cout << "ft.size " << ft.size() << endl;
    
    int desc_id = rand() % (int)ft.size();
    map<string, Matrix>::iterator it = ft.begin();
    //cout << desc_id << " desc id" << endl;
    for(int i=0; i<desc_id; i++) {
      //cout << i << " " << ft.size() << endl;
      it++;
    }
    Matrix& v = (*it).second;
    string desc = (*it).first;

    //Get a random feature.
    int feature_id = (rand() % (int)v.Ncols()) + 1;

    //Make the weak classifier.
    weak_classifier wc;
    wc.descriptor = desc;
    wc.center = v.Column(feature_id);
    cand.push_back(wc);
    //cout << "Added a new candidate: " << wc.descriptor << " with feature id " << feature_id << " from obj " << obj_id << endl;
  }
  cout << "Added " << nCandidates << " candidate wcs" << endl;

  // -- For all candidates, try several thetas and get their utilities.
  Matrix *best_weights, *pweights=NULL;
  Matrix **ppweights = &pweights;
  float objective = computeObjective();
  float max_util = 0.0;
  Matrix best_mmt;
  for(unsigned int i=0; i<cand.size(); i++) {
    vector<Matrix> dists;

    for(unsigned int j=0; j<dd_->objs_.size(); j++) {
      Matrix &f = dd_->objs_[j].features[cand[i].descriptor];
      dists.push_back(Matrix(1, f.Ncols()));  
      dists[j] = 0.0;
      for(int k=1; k<=f.Ncols(); k++) {
	dists[j](1, k) = euc(f.Column(k), cand[i].center);
      }
    }

//     for(unsigned int j=0; j<dd_->objs_.size(); j++) {
//       cout << "*** obj " << j << endl;
//       cout << dists[j] << endl;
//     }

    for(int iTheta=0; iTheta<nThetas; iTheta++) {
    
      //For now, take thetas between 0 and .5sqrt(dimensionality) of the feature space uniformly.
      //This requires that the features all be normalized so that the max element is +1 and the min is 0.
      Matrix& f = dd_->objs_[0].features[cand[i].descriptor];
      cand[i].theta = ((float)rand() / (float)RAND_MAX) * (1.0/2.0) * sqrt(f.Nrows());
      //cout << "Max theta is " << (1.0/2.0) * sqrt(f.Nrows()) << endl;
      //cout << "Using theta of " << theta << endl;

      //Get the M_m^t, number of points in the hypersphere for each object.
      Matrix mmt(1,dd_->objs_.size()); mmt=0.0;
      for(unsigned int j=0; j<dists.size(); j++) {
	for(int k=1; k<=dists[j].Ncols(); k++) {
	  if(dists[j](1,k) < cand[i].theta)
	    mmt(1, j+1)++;
	}
      }

      //cout << mmt << endl;

      // -- Use Newton's method to solve for the a_t^c's.
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

      for(unsigned int c=0; c<dd_->nClasses_; c++) {
	float numerator = 0.0;
	float denominator = 0.0;
	for(unsigned int m=0; m<dd_->objs_.size(); m++) {
	  numerator += weights_(c+1, m+1) * dd_->ymc_(c+1, m+1) * mmt(1, m+1);
	  denominator += weights_(c+1, m+1) * mmt(1, m+1) * mmt(1, m+1);
	}
	if(denominator==0)
	  cand[i].vals(c+1,1) = 0;
	else
	  cand[i].vals(c+1,1) = numerator / denominator;
      }

      //Get the utility.
      float new_objective = computeNewObjective(cand[i], mmt, ppweights);
      float util = objective - new_objective;

      //cout << "  Theta " << cand[i].theta << ": objective " << objective << ",  new_objective " << new_objective << ", max_util " << max_util << ", util " << util;
      //If this is the best wc so far, save it.
      if(util >= max_util) {
	//cout << "***" << endl;
	max_util = util;
	best = cand[i];
	best_weights = *ppweights;
	best_mmt = mmt;
      }
      else {
	delete *ppweights; *ppweights = NULL;
	//cout << endl;
      }

    }
    //cout << "WC " << i <<": Tried " << nThetas << " thetas, max utility is " << max_util << endl;
    cout << "."; cout.flush();
  }
  cout << endl;
  // -- Save the best weak classifier in the battery.
  best.id = nWCs_++;
  battery_[best.descriptor].push_back(best);

  //cout << dd_->ymc_;
  time(&end);
  //cout << dd_->status();
  cout << "Took " << difftime(end,start) << " seconds to try " << nCandidates << " wcs with " << nThetas << " thetas each." << endl;
  cout << displayWeakClassifier(best) << endl;
  int nObjs_encompassed = 0;
  for(int i=1; i<=best_mmt.Ncols(); i++) {
    if(best_mmt(1,i) != 0)
      nObjs_encompassed++;
  }
  cout << "WC encompasses at least one point from " << nObjs_encompassed << " out of " << best_mmt.Ncols() << " objects." << endl;

  // -- Update the weights.
  //  cout << weights_ << endl;
  //  cout << *best_weights << endl;
  weights_ = *best_weights;
  delete *ppweights; *ppweights = NULL;
}


string displayWeakClassifier(const weak_classifier &wc) {
  ostringstream oss(ostringstream::out);
  oss << wc.descriptor << " descriptor." << endl;
  oss << "Theta: " << wc.theta << endl;
  oss << "Vals: " << endl << wc.vals;
  return oss.str();
}


float Dorylus::computeObjective() {
  return weights_.Sum();
}

//mmt is a M_m^t specific to this weak classifier and this dataset.
Matrix Dorylus::computeDatasetActivations(const weak_classifier& wc, const Matrix& mmt) {
  Matrix act(dd_->nClasses_, dd_->objs_.size()); act=0.0;
  
  for(unsigned int m=0; m<dd_->objs_.size(); m++) {

    //This can become an outer product.
    for(unsigned int c=0; c<dd_->nClasses_; c++) {
      act(c+1, m+1) += wc.vals(c+1,1) * mmt(1, m+1);
    }
  }

  return act;
}


float Dorylus::computeNewObjective(const weak_classifier& wc, const Matrix& mmt, Matrix** new_weights) {
  Matrix weights = weights_;
  Matrix act = computeDatasetActivations(wc, mmt);

  for(unsigned int c=0; c<dd_->nClasses_; c++) {
    for(unsigned int m=0; m<dd_->objs_.size(); m++) {
      weights(c+1, m+1) = weights(c+1, m+1) * exp(-dd_->ymc_(c+1, m+1) * act(c+1,m+1));
      if(weights(c+1, m+1) < 0) {
	cerr << "One of the weights is negative!" << endl;
	exit(0);
      }
      if(weights(c+1, m+1) == 0) {
	cerr << "One of the weights is zero!" << endl;
	weights(c+1, m+1) = FLT_MIN;
      }
    }
  }

  if(new_weights != NULL) {
    *new_weights = new Matrix;
    **new_weights = weights;
  }

  return weights.Sum();
}

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
