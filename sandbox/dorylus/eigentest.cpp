#include <dorylus.h>

USING_PART_OF_NAMESPACE_EIGEN;
using namespace std;

int main(int argc, char** argv) {
  VectorXf x;
  x = VectorXf::Ones(10);
  cout << x.rows() << endl;

  cout << x.cwise().exp().transpose() << endl;
  cout << x.transpose() << endl;
}
