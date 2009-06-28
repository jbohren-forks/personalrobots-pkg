#include <dorylus.h>
using namespace NEWMAT;
using namespace std;

void testDatasetSave()
{
  cout << "Running save test." << endl;
  DorylusDataset dd;

  map<string, Matrix*> f;
  Matrix* spin = new Matrix(3,2);
  (*spin)(1,1) = 1;
  (*spin)(2,1) = 2;
  (*spin)(3,1) = 3;
  (*spin)(1,2) = 4;
  (*spin)(2,2) = 5;
  (*spin)(3,2) = 6;
  Matrix* sift = new Matrix(3,2); (*sift) = 2.2;
  (*sift)(1,1) = 101;
  (*sift)(2,2) = 102;
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

  cout << dd2.displayYmc() << endl;
  cout << dd2.displayFeatures() << endl;

}

int main(int argc, char** argv) {
  testDatasetSave();
}
