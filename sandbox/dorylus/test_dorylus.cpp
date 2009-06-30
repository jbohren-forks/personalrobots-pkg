#include <dorylus.h>
//using namespace NEWMAT;
USING_PART_OF_NAMESPACE_EIGEN;
using namespace std;

void testDatasetSave()
{
  cout << "Running save test." << endl;
  DorylusDataset dd;

  map<string, MatrixXf*> f;
  MatrixXf* spin = new MatrixXf(3,2);
  (*spin)(0,0) = 1;
  (*spin)(1,0) = 2;
  (*spin)(2,0) = 3;
  (*spin)(0,1) = 4;
  (*spin)(1,1) = 5;
  (*spin)(2,1) = 6;
  MatrixXf* sift = new MatrixXf(2,2); 
  (*sift)(0,0) = 101;
  (*sift)(0,1) = 10;
  (*sift)(1,0) = 13;
  (*sift)(1,1) = 102;
  f["spinimg"] = spin;
  f["sift"] = sift;

  object obj;
  obj.label = 0;
  obj.features = f;
  //cout << displayObject(obj) << endl;
  vector<object*> objs;
  objs.push_back(&obj);
  obj.label = 1;
  //cout << displayObject(obj) << endl;
  objs.push_back(&obj);
  //cout << displayObject(obj) << endl;


  dd.setObjs(objs);

  cout << "*** BEFORE:" << endl;
  cout << dd.status() << endl;
  cout << dd.displayYmc() << endl;
  cout << dd.displayObjects() << endl;

  dd.save(string("test.dd"));

  DorylusDataset dd2;
  dd2.load(string("test.dd"));
  dd2.save(string("test2.dd"));
  cout << "*** AFTER:" << endl;
  cout << dd2.status() << endl;
  cout << dd2.displayYmc() << endl;
  cout << dd2.displayObjects() << endl;

}

int main(int argc, char** argv) {
  testDatasetSave();
}
