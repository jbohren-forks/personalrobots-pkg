#ifndef _intersection_sphere_h_
#define _intersection_sphere_h_

namespace scan_utils {

bool boxSphereTest ( const float boxcenter[3], const float boxhalfsize[3],
		     const float spherecenter[3], float radius);
} //namespace scane utils

#endif
