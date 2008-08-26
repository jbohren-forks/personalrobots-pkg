
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
  oss << "ymc_: " << endl << ymc_ << endl;
  oss << "object data" << endl << displayFeatures() << endl;
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
      for (int j=0; j<nRows; j++) {
	for(int i=0; i<nCols; i++) {
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
	  
  return true; 
}


// int main(int argc, char **argv)
// {
//   DorylusDataset dd;

//   cout << sizeof(Real) << " " << sizeof(Matrix) << endl;

//   map<string, Matrix> f;
//   Matrix spin(3,1); spin = 1.0;
//   Matrix sift(2,1); sift = 2.2;
//   f["spinimg"] = spin;
//   f["sift"] = sift;

//   object obj;
//   obj.label = 0;
//   obj.features = f;
//   vector<object> objs;
//   objs.push_back(obj);
//   dd.setObjs(objs);

//   cout << dd.status() << endl;
//   dd.save(string("test.dd"));
  
//   DorylusDataset dd2;
//   dd2.load(string("test.dd"));
//   cout << dd2.status() << endl;
//   return 0;
// }
    
