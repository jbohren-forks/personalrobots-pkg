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
#include "intersection_triangle.h"
#include <math.h>

/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-Möller                              */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/
/* Downloaded by Matei Ciocarlie from                   */
/* http://jgt.akpeters.com/papers/AkenineMoller01/tribox.html */
/********************************************************/

/*
  Macros are used to speed things up - speed is very important here. I
  have prefaced all macros with OI_ (standing for Octree Intersections)
  to avoid namespace collisions.
 */

#define OI_X 0
#define OI_Y 1
#define OI_Z 2

#define OI_CROSS(dest,v1,v2) \
	dest[0]=v1[1]*v2[2]-v1[2]*v2[1];   \
	dest[1]=v1[2]*v2[0]-v1[0]*v2[2];   \
	dest[2]=v1[0]*v2[1]-v1[1]*v2[0]; 

#define OI_DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define OI_SUB(dest,v1,v2) \
	dest[0]=v1[0]-v2[0];   \
	dest[1]=v1[1]-v2[1];   \
	dest[2]=v1[2]-v2[2]; 

#define OI_FINDMINMAX(x0,x1,x2,min,max) \
	min = max = x0;			\
	if(x1<min) min=x1;		\
	if(x1>max) max=x1;		\
	if(x2<min) min=x2;		\
	if(x2>max) max=x2;


/*======================== X-tests ========================*/
#define OI_AXISTEST_X01(a, b, fa, fb)			   \
	p0 = a*v0[OI_Y] - b*v0[OI_Z];			   \
	p2 = a*v2[OI_Y] - b*v2[OI_Z];			   \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[OI_Y] + fb * boxhalfsize[OI_Z];   \
	if(min>rad || max<-rad) return 0;

#define OI_AXISTEST_X2(a, b, fa, fb)			   \
	p0 = a*v0[OI_Y] - b*v0[OI_Z];			   \
	p1 = a*v1[OI_Y] - b*v1[OI_Z];			   \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[OI_Y] + fb * boxhalfsize[OI_Z];   \
	if(min>rad || max<-rad) return 0;

/*======================== Y-tests ========================*/
#define OI_AXISTEST_Y02(a, b, fa, fb)			   \
	p0 = -a*v0[OI_X] + b*v0[OI_Z];		      	   \
	p2 = -a*v2[OI_X] + b*v2[OI_Z];	       	       	   \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
	rad = fa * boxhalfsize[OI_X] + fb * boxhalfsize[OI_Z];   \
	if(min>rad || max<-rad) return 0;

#define OI_AXISTEST_Y1(a, b, fa, fb)			   \
	p0 = -a*v0[OI_X] + b*v0[OI_Z];		      	   \
	p1 = -a*v1[OI_X] + b*v1[OI_Z];	     	       	   \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[OI_X] + fb * boxhalfsize[OI_Z];   \
	if(min>rad || max<-rad) return 0;

/*======================== Z-tests ========================*/

#define OI_AXISTEST_Z12(a, b, fa, fb)			   \
	p1 = a*v1[OI_X] - b*v1[OI_Y];			   \
	p2 = a*v2[OI_X] - b*v2[OI_Y];			   \
        if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
	rad = fa * boxhalfsize[OI_X] + fb * boxhalfsize[OI_Y];   \
	if(min>rad || max<-rad) return 0;

#define OI_AXISTEST_Z0(a, b, fa, fb)			   \
	p0 = a*v0[OI_X] - b*v0[OI_Y];			   \
	p1 = a*v1[OI_X] - b*v1[OI_Y];			   \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
	rad = fa * boxhalfsize[OI_X] + fb * boxhalfsize[OI_Y];   \
	if(min>rad || max<-rad) return 0;

namespace scan_utils{

int planeBoxOverlap(float normal[3],float d, float maxbox[3])
{
  int q;
  float vmin[3],vmax[3];
  for(q=OI_X;q<=OI_Z;q++)
  {
    if(normal[q]>0.0f)
    {
      vmin[q]=-maxbox[q];
      vmax[q]=maxbox[q];
    }
    else
    {
      vmin[q]=maxbox[q];
      vmax[q]=-maxbox[q];
    }
  }
  if(OI_DOT(normal,vmin)+d>0.0f) return 0;
  if(OI_DOT(normal,vmax)+d>=0.0f) return 1;
  
  return 0;
}

int triBoxOverlap(float boxcenter[3],float boxhalfsize[3],
		  float trivert0[3], float trivert1[3], float trivert2[3])
{
	/*
	fprintf(stderr,"Test:\nbox: %f %f %f\nhalfsize: %f %f %f\n",
		boxcenter[0], boxcenter[1], boxcenter[2],
		boxhalfsize[0], boxhalfsize[1], boxhalfsize[2]);
	fprintf(stderr,"Vert0: %f %f %f\nVert1: %f %f %f\nVert2: %f %f %f\n",
		trivert0[0], trivert0[1], trivert0[2],
		trivert1[0], trivert1[1], trivert1[2],
		trivert2[0], trivert2[1], trivert2[2]);
	*/

  /*    use separating axis theorem to test overlap between triangle and box */
  /*    need to test for overlap in these directions: */
  /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
  /*       we do not even need to test these) */
  /*    2) normal of the triangle */
  /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
  /*       this gives 3x3=9 more tests */
   float v0[3],v1[3],v2[3];
   // float axis[3]; //this is not used
   float min,max,d,p0,p1,p2,rad,fex,fey,fez;  
   float normal[3],e0[3],e1[3],e2[3];

   /* This is the fastest branch on Sun */
   /* move everything so that the boxcenter is in (0,0,0) */
   OI_SUB(v0,trivert0,boxcenter);
   OI_SUB(v1,trivert1,boxcenter);
   OI_SUB(v2,trivert2,boxcenter);

   /* compute triangle edges */
   OI_SUB(e0,v1,v0);      /* tri edge 0 */
   OI_SUB(e1,v2,v1);      /* tri edge 1 */
   OI_SUB(e2,v0,v2);      /* tri edge 2 */

   /* Bullet 3:  */
   /*  test the 9 tests first (this was faster) */
   fex = fabs(e0[OI_X]);
   fey = fabs(e0[OI_Y]);
   fez = fabs(e0[OI_Z]);
   OI_AXISTEST_X01(e0[OI_Z], e0[OI_Y], fez, fey);
   OI_AXISTEST_Y02(e0[OI_Z], e0[OI_X], fez, fex);
   OI_AXISTEST_Z12(e0[OI_Y], e0[OI_X], fey, fex);

   fex = fabs(e1[OI_X]);
   fey = fabs(e1[OI_Y]);
   fez = fabs(e1[OI_Z]);
   OI_AXISTEST_X01(e1[OI_Z], e1[OI_Y], fez, fey);
   OI_AXISTEST_Y02(e1[OI_Z], e1[OI_X], fez, fex);
   OI_AXISTEST_Z0(e1[OI_Y], e1[OI_X], fey, fex);

   fex = fabs(e2[OI_X]);
   fey = fabs(e2[OI_Y]);
   fez = fabs(e2[OI_Z]);
   OI_AXISTEST_X2(e2[OI_Z], e2[OI_Y], fez, fey);
   OI_AXISTEST_Y1(e2[OI_Z], e2[OI_X], fez, fex);
   OI_AXISTEST_Z12(e2[OI_Y], e2[OI_X], fey, fex);

   /* Bullet 1: */
   /*  first test overlap in the {x,y,z}-directions */
   /*  find min, max of the triangle each direction, and test for overlap in */
   /*  that direction -- this is equivalent to testing a minimal AABB around */
   /*  the triangle against the AABB */

   /* test in X-direction */
   OI_FINDMINMAX(v0[OI_X],v1[OI_X],v2[OI_X],min,max);
   if(min>boxhalfsize[OI_X] || max<-boxhalfsize[OI_X]) return 0;

   /* test in Y-direction */
   OI_FINDMINMAX(v0[OI_Y],v1[OI_Y],v2[OI_Y],min,max);
   if(min>boxhalfsize[OI_Y] || max<-boxhalfsize[OI_Y]) return 0;

   /* test in Z-direction */
   OI_FINDMINMAX(v0[OI_Z],v1[OI_Z],v2[OI_Z],min,max);
   if(min>boxhalfsize[OI_Z] || max<-boxhalfsize[OI_Z]) return 0;

   /* Bullet 2: */
   /*  test if the box intersects the plane of the triangle */
   /*  compute plane equation of triangle: normal*x+d=0 */
   OI_CROSS(normal,e0,e1);
   d=-OI_DOT(normal,v0);  /* plane eq: normal.x+d=0 */
   if(!planeBoxOverlap(normal,d,boxhalfsize)) return 0;

   return 1;   /* box and triangle overlaps */
}

} //namespace scan_utils
