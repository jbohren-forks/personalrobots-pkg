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
#include "intersection_obb.h"

#include <math.h>

/*! \file Intersection test for OBB - OBB. Optimized to also handle
    the AABB - OBB case reasonably well.

    Original code by Miguel Gomez, published in a Gamasutra article.

    Downloaded from http://www.gamasutra.com/features/19991018/Gomez_5.htm

    Small modifications have been made to the downloaded code to
    handle the AABB - OBB case a little better.

 */

namespace scan_utils {

/*! Performs intersection test between two OBB's.

  \param Pa,Pb - centers of boxes A and B

  \param a,b - extents of boxes A and B along their respective axes

  \param A,B - directions of the three axes of each box, as an
  orthonormal basis.

  If A is an AABB, pass \a NULL instead of the parameter \a A.
 */
bool boxIntersectionTest(const float* Pa, const float *a, const float A[][3], //position, extents and orthonormal basis
			 const float* Pb, const float *b, const float B[][3]) //position, extents and orthonormal basis
{
	/*
	fprintf(stderr,"Test\n");
	fprintf(stderr,"A position: %f %f %f \n",Pa[0], Pa[1], Pa[2]);
	fprintf(stderr,"A extents : %f %f %f \n",a[0], a[1], a[2]);
	fprintf(stderr,"B position: %f %f %f \n",Pb[0], Pb[1], Pb[2]);
	fprintf(stderr,"B extents : %f %f %f \n",b[0], b[1], b[2]);
	fprintf(stderr,"B axes:\n");
	fprintf(stderr,"%f %f %f \n",B[0][0], B[0][1], B[0][2]);
	fprintf(stderr,"%f %f %f \n",B[1][0], B[1][1], B[1][2]);
	fprintf(stderr,"%f %f %f \n",B[2][0], B[2][1], B[2][2]);
	*/
	float T[3];

	float R[3][3];
	float ra, rb, t;
	int i, k;

	//translation between box centers
	T[0] = Pb[0] - Pa[0];
	T[1] = Pb[1] - Pa[1];
	T[2] = Pb[2] - Pa[2];
		
	if (A) {
		//A is not AABB. Compute rotation matrix between A and B
		for( i=0 ; i<3 ; i++ ) {
			for( k=0 ; k<3 ; k++ ) {
				//R[i,k] = A[i] dot B[k]
				R[i][k] = A[i][0] * B[i][0] + A[i][1] * B[i][1] + A[i][2] * B[i][2];
			}
		}

		//also rotate vector between box centers
		float tmp0 = T[0] * A[0][0] + T[1] * A[0][1] + T[2] * A[0][2];
		float tmp1 = T[0] * A[1][0] + T[1] * A[1][1] + T[2] * A[1][2];
		float tmp2 = T[0] * A[2][0] + T[1] * A[2][1] + T[2] * A[2][2];
		T[0] = tmp0; T[1] = tmp1; T[2] = tmp2;

	} else {
		// A is an AABB. Rotation matrix is equal to B.
		for (i=0; i<3; i++) {
			for (k=0; k<3; k++) {
				R[i][k] = B[i][k];
			}
		}
		// and translation vector T is already aligned
	}

	/*ALGORITHM: Use the separating axis test for all 15 potential
	  separating axes. If a separating axis could not be found, the two
	  boxes overlap. */
	
	//A's basis vectors
	for( i=0 ; i<3 ; i++ ){
		
		ra = a[i];
			
                rb = b[0]*fabs(R[i][0]) + b[1]*fabs(R[i][1]) + b[2]*fabs(R[i][2]);

                t = fabs( T[i] );
		
                if( t > ra + rb )
			return false;
		
	}

	//B's basis vectors
	for( k=0 ; k<3 ; k++ ){

                ra = a[0]*fabs(R[0][k]) + a[1]*fabs(R[1][k]) + a[2]*fabs(R[2][k]);

                rb = b[k];

                t = fabs( T[0]*R[0][k] + T[1]*R[1][k] + T[2]*R[2][k] );
		
                if( t > ra + rb )
			return false;
		
	}

	//9 cross products
	
	//L = A0 x B0
	ra = a[1]*fabs(R[2][0]) + a[2]*fabs(R[1][0]);

	rb =
		b[1]*fabs(R[0][2]) + b[2]*fabs(R[0][1]);
	
	t =
		fabs( T[2]*R[1][0] -
		      T[1]*R[2][0] );
	
	if( t > ra + rb )
		return false;
	
	//L = A0 x B1
	ra =
		a[1]*fabs(R[2][1]) + a[2]*fabs(R[1][1]);
	
	rb =
		b[0]*fabs(R[0][2]) + b[2]*fabs(R[0][0]);
	
	t =
		fabs( T[2]*R[1][1] -
		      T[1]*R[2][1] );
	
	if( t > ra + rb )
		return false;
	
	//L = A0 x B2
	ra =
		a[1]*fabs(R[2][2]) + a[2]*fabs(R[1][2]);
	
	rb =
		b[0]*fabs(R[0][1]) + b[1]*fabs(R[0][0]);
	
	t =
		fabs( T[2]*R[1][2] -
		      T[1]*R[2][2] );
	
	if( t > ra + rb )
		return false;
	
	//L = A1 x B0
	ra =
		a[0]*fabs(R[2][0]) + a[2]*fabs(R[0][0]);
	
	rb =
		b[1]*fabs(R[1][2]) + b[2]*fabs(R[1][1]);
	
	t =
		fabs( T[0]*R[2][0] -
		      T[2]*R[0][0] );
	
	if( t > ra + rb )
		return false;
	
	//L = A1 x B1
	ra =
		a[0]*fabs(R[2][1]) + a[2]*fabs(R[0][1]);
	
	rb =
		b[0]*fabs(R[1][2]) + b[2]*fabs(R[1][0]);
	
	t =
		fabs( T[0]*R[2][1] -
		      T[2]*R[0][1] );
	
	if( t > ra + rb )
		return false;
	
	//L = A1 x B2
	ra =
		a[0]*fabs(R[2][2]) + a[2]*fabs(R[0][2]);
	
	rb =
		b[0]*fabs(R[1][1]) + b[1]*fabs(R[1][0]);
	
	t =
		fabs( T[0]*R[2][2] -
		      T[2]*R[0][2] );
	
	if( t > ra + rb )
		return false;
	
	//L = A2 x B0
	ra =
		a[0]*fabs(R[1][0]) + a[1]*fabs(R[0][0]);
	
	rb =
		b[1]*fabs(R[2][2]) + b[2]*fabs(R[2][1]);
	
	t =
		fabs( T[1]*R[0][0] -
		      T[0]*R[1][0] );
	
	if( t > ra + rb )
		return false;
	
	//L = A2 x B1
	ra =
		a[0]*fabs(R[1][1]) + a[1]*fabs(R[0][1]);
	
	rb =
		b[0] *fabs(R[2][2]) + b[2]*fabs(R[2][0]);
	
	t =
		fabs( T[1]*R[0][1] -
		      T[0]*R[1][1] );
	
	if( t > ra + rb )
		return false;
	
	//L = A2 x B2
	ra =
		a[0]*fabs(R[1][2]) + a[1]*fabs(R[0][2]);
	
	rb =
		b[0]*fabs(R[2][1]) + b[1]*fabs(R[2][0]);
	
	t =
		fabs( T[1]*R[0][2] -
		      T[0]*R[1][2] );
	
	if( t > ra + rb )
		return false;
	
	/*no separating axis found,
	  the two boxes overlap */
	
	return true;
	
}

} //namespace scan_utils
