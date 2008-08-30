#include "util/uBLASUtils.hh"

std::ostream& operator<<(std::ostream &sout, 
			 const boost::numeric::ublas::matrix<double>& mat) {
  
  sout << "Matrix " << mat.size1() << "x" << mat.size2() << " [ " << std::endl;
  for (int row = 0; row < (int)mat.size1(); row++) {
    for (int col = 0; col < (int)mat.size2(); col++) {
      sout << mat(row,col) << " ";
    }
    sout << ";" << std::endl;
  }
  sout << " ]" << std::endl;

  return sout;
}

std::ostream& operator<<(std::ostream& sout,
			 const boost::numeric::ublas::vector<double>& vec) {
  sout << "Vector " << vec.size() << " [ " << std::endl;
  for (int ii = 0; ii < (int)vec.size(); ii++) {
    sout << vec(ii) << " ";
  }
  sout << " ]" << std::endl;
  return sout;
}
