#include "features/FeatureMatrix.hh"

std::ostream& operator<<(std::ostream& sout, 
			 const DenseFeatureMatrix<double>& fmat) {
  assert(fmat.fmat != NULL);
  return sout << *(fmat.fmat);
}
