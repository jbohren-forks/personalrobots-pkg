#ifndef __UBLAS_UTILS_H__
#define __UBLAS_UTILS_H__

#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>

// FIXME: templatize or something
std::ostream& operator<<(std::ostream &sout, 
			 const boost::numeric::ublas::matrix<double>& mat);

std::ostream& operator<<(std::ostream &sout, 
			 const boost::numeric::ublas::vector<double>& mat);

	   /*
namespace uBLASUtils {
};
	   */

#endif
