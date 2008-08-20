#ifndef _intersection_obb_h_
#define _intersection_obb_h_

namespace scan_utils {
bool boxIntersectionTest(const float* Pa, const float *a, const float A[][3], //position, extents and orthonormal basis
			 const float* Pb, const float *b, const float B[][3]); //position, extents and orthonormal basis

} //namespace scan_utils

#endif
