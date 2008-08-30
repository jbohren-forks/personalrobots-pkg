#ifndef __DVEC_H__
#define __DVEC_H__

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <math.h>

// FIXME: think of a better solution
/**
   @brief A wrapper class for ublas vector.  

   Currently exists 
   mainly to clear vector on construction, which ublas 
   does not do by default.
 */
class Dvec : public boost::numeric::ublas::vector<double> {
public:
  /**
     @brief Zero-initializes on construction
     @param len The length of the vector to be constructed
  */
  Dvec(int len) :
    boost::numeric::ublas::vector<double>(len)
  {
    clear();
  }

  template<class AE>
  Dvec(const boost::numeric::ublas::vector_expression<AE> &ae) :
    boost::numeric::ublas::vector<double>(ae) 
  {
  }

  void assertFinite() const {
    for (int ii = 0; ii < (int)size(); ii++)
      assert(finite((*this)(ii)));
  }

  using boost::numeric::ublas::vector<double>::operator=;
};

/**
   @brief A wrapper class for ublas vector_range
 */
class DvecView : public boost::numeric::ublas::vector_range<Dvec> {
public:
  DvecView(Dvec& vec, const boost::numeric::ublas::range& range) :
    boost::numeric::ublas::vector_range<Dvec>(vec, range)
  {
  }

  using boost::numeric::ublas::vector_range<Dvec>::operator=;
};

/**
   @brief A wrapper class for a const ublas vector_range
 */
class DvecViewConst : public boost::numeric::ublas::vector_range<const Dvec> {
public:
  DvecViewConst(const Dvec& vec, const boost::numeric::ublas::range& range) :
    boost::numeric::ublas::vector_range<const Dvec>(vec, range)
  {
  }

  using boost::numeric::ublas::vector_range<const Dvec>::operator=;
};

typedef boost::numeric::ublas::range Drange;
//typedef boost::numeric::ublas::vector<double> Dvec;
//typedef boost::numeric::ublas::vector_range<Dvec> DvecView;
//typedef const boost::numeric::ublas::vector_range<const Dvec> DvecViewConst;

/**
   @brief Utilities relating to Dvec's
   @todo Should probably roll this into Dvec wrapper class
 */
namespace DvecUtils {
  /**
     @brief Serializes a Dvec
     @param vec The vector to be serialized
     @param ostr An output stream to which to serialize
   */
  void serialize(const Dvec& vec, std::ostream& ostr);

  /**
     @brief Deserializes a Dvec
     @param istr Stream from which to deserialize
     @return the deserialized Dvec
   */
  Dvec* deserialize(std::istream& istr);

  /**
     @brief Returns a somewhat-randomly-initialized Dvec 
     @param size The size of the vector
     @return a random vector
     @todo make an argument that specifies possible range of values
   */
  Dvec randomVector(int size);

};

#endif
