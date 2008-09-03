
#include<dorylus.h>

using namespace std;
#define Matrix NEWMAT::Matrix
//using namespace NEWMAT;

void DorylusDataset::setObjs(const vector<object>& objs) 
{
  objs_ = objs;

  // -- Get number of classes.
  for(unsigned int i=0; i<objs_.size(); i++) {
    class_labels_[objs_[i].label]++;
  }
  nClasses_ = class_labels_.size();
  
  // -- Build ymc_
  ymc_ = Matrix(nClasses_, objs_.size()); ymc_=0.0;
  for(unsigned int i=0; i<objs_.size(); i++) {
    for(int j=0; j<(int)nClasses_; j++) {
      if(objs[i].label==j)
	ymc_(j+1, i+1) = 1;
      else
	ymc_(j+1, i+1) = -1;
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
//   oss << "  class_labels_: " << "\n";
//   oss << "       " << "\n";
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

  int nRows, nCols;
  string descriptor;
  object *pobj = NULL;
  while(true) {

    getline(f, line);

    if(line.size() == 0) {
      if(pobj) {
	objs_.push_back(*pobj);
	delete pobj; pobj = NULL;
      }	
      else {
	cerr << "no object??" << endl;
      }
      break;
    }

    else if(line.compare(string("New object.")) == 0) {
      if(pobj) {
	objs_.push_back(*pobj);
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

  // -- Reconstruct ymc_
  ymc_ = Matrix(nClasses_, objs_.size());
  for(unsigned int c=0; c<objs_.size(); c++) {
    for(unsigned int r=0; r<nClasses_; r++) {
      if(objs_[c].label == (int)r)
	ymc_(r+1,c+1) = 1;
      else
	ymc_(r+1,c+1) = -1;
    }
  }

  // -- Get nClasses_ and class_labels_.
  for(unsigned int c=0; c<objs_.size(); c++) {
    class_labels_[objs_[c].label] = 0;
  }
  for(unsigned int c=0; c<objs_.size(); c++) {
    class_labels_[objs_[c].label]++;
  }
  nClasses_ = class_labels_.size();
  
	  
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
    
void Dorylus::loadDataset(DorylusDataset *dd) {
  dd_ = dd;
  weights_ = Matrix(dd_->nClasses_, dd_->objs_.size());


  for(int i=1; i<=weights_.Nrows(); i++) {
    for(int j=1; j<=weights_.Ncols(); j++) {
      weights_(i,j) = 1;
    }
  }
 
 normalizeWeights();
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
	
void Dorylus::learnWC(int nCandidates, float maxErr) {
  int nThetas = 10;
  assert(dd_!=0);

  // -- Choose wc candidates from the distribution of weights over the objects.

  vector<weak_classifier> cand;
  for(int iCand=0; iCand<nCandidates; iCand++) {
    cout << "Constructing candidate " << iCand << endl;
    float dice = (float)rand() / (float)RAND_MAX;
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
    cout << "Picking from obj " << obj_id << endl;

    //Get a random descriptor.
    map<string, Matrix> &ft = dd_->objs_[obj_id].features;
    //cout << "ft.size " << ft.size() << endl;
    
    int desc_id = rand() % (int)ft.size();
    map<string, Matrix>::iterator it = ft.begin();
    //cout << desc_id << " desc id" << endl;
    for(int i=0; i<desc_id; i++) {
      it++;
    }
    Matrix& v = (*it).second;
    string desc = (*it).first;

    //Get a random feature.
    int feature_id = rand() % (int)v.Ncols();

    //Make the weak classifier.
    weak_classifier wc;
    wc.descriptor = desc;
    wc.center = v.Column(feature_id);
    cand.push_back(wc);
    cout << "Added a new candidate: " << wc.descriptor << " with feature id " << feature_id << " from obj " << obj_id << endl;
  }

  // -- For all candidates, try several thetas and get their utilities.
  
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

    //   float objective = computeObjective();
    float max_util = 0.0;
    for(int iTheta=0; iTheta<nThetas; iTheta++) {
    
      //For now, take thetas between 0 and .5sqrt(dimensionality) of the feature space uniformly.
      //This requires that the features all be normalized so that the max element is +1 and the min is 0.
      Matrix& f = dd_->objs_[0].features[cand[i].descriptor];
      float theta = ((float)rand() / (float)RAND_MAX) * (1.0/2.0) * sqrt(f.Nrows());
      cout << "Max theta is " << (1.0/2.0) * sqrt(f.Nrows()) << endl;
      cout << "Using theta of " << theta << endl;

      //Get the M_m^t, number of points in the hypersphere for each object.
      Matrix mmt(1,dd_->objs_.size()); mmt=0;
      for(unsigned int j=0; j<dists.size(); j++) {
	for(int k=1; k<dists[j].Ncols(); k++) {
	  if(dists[j](1,k) < theta)
	    mmt(1, j+1) = mmt(1, j+1) + 1;
	}
      }

      cout << mmt << endl;

//       //Use Newton's method to solve for the a_t^c's.
//       temp_wc = cand[i];
//       temp_wc.vals = newton;

//       //Get the utility.
//       float new_objective = computeNewObjective(temp_wc);
//       float util = objective - new_objective;
      
//       //If this is the best theta so far, save it.
//       if(util > max_util) {
// 	max_util = util;
// 	cand[i] = temp_wc;
//       }
    }
  }
}
  

