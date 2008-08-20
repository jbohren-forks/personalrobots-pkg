#include "intersection_sphere.h"
#include <math.h>

/*! \file Intersection test for AABB - Sphere

    Original code by Miguel Gomez, published in a Gamasutra article.

    Downloaded from http://www.gamasutra.com/features/19991018/Gomez_4.htm
*/

namespace scan_utils {

/*! Performs intersection test between an AABB and a sphere

 */
//Check to see if the sphere overlaps the AABB
bool boxSphereTest ( const float boxcenter[3], const float boxhalfsize[3],
		     const float spherecenter[3], float radius)
{
	float s, d = 0; 	
	//find the square of the distance from the sphere to the box
	for( int i=0 ; i<3 ; i++ ){
		if( spherecenter[i] < boxcenter[i] - boxhalfsize[i] ){
			s = spherecenter[i] - (boxcenter[i] - boxhalfsize[i]);
			d += s*s; 
		} else if( spherecenter[i] > boxcenter[i] + boxhalfsize[i] ){
			s = spherecenter[i] - (boxcenter[i] + boxhalfsize[i]);
			d += s*s; 			
		}
		
	}
	return d <= radius*radius;	
}
} //namespace scane utils
