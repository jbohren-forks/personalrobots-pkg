#include "util/Dvec.hh"

namespace DvecUtils {

  // exists to be able to serialize regular Dvecs
  void serialize(const Dvec& vec, std::ostream& ostr) {
    ostr << "dvec " << vec.size() << std::endl;
    //    ostr << (*this) << std::endl;
    for (int ii = 0; ii < (int)vec.size(); ii++) {
      ostr << vec(ii) << std::endl;
    }
  }

  Dvec* deserialize(std::istream& istr) {
    int len = 0;
    std::string sbuf;
    istr >> sbuf;
    //    sscanf(sbuf.c_str(), "dvec %d", &len);

    std::cout << "DVEC " << sbuf.c_str() << std::endl;

    istr >> len;

    Dvec *dvec0 = new Dvec(len);

    for (int ii = 0; ii < len; ii++) {
      double val;
      if (! (istr >> val)) 
	throw "Deserialization error";
      (*dvec0)(ii) = val;
    }

    //    cerr << "Deserialized dvec0: " << *dvec0 << std::endl;
    
    return dvec0;
  }

  Dvec randomVector(int size) {
    Dvec testVec(size);
    for (int col = 0; col < size; col++) 
      testVec(col) = 1e6 * drand48();
    return testVec;
  }

};
