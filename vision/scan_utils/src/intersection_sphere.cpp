/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Matei Ciocarlie */


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
