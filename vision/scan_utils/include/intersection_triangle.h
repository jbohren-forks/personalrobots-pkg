#ifndef _intersections_h_
#define _intersections_h_

/*! \file

  Intersection test for AABB - triangle.
*/

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


namespace scan_utils{

int planeBoxOverlap(float normal[3],float d, float maxbox[3]);
int triBoxOverlap(float boxcenter[3],float boxhalfsize[3],
			 float trivert0[3], float trivert1[3], float trivert2[3]);

} //namespace scan_utils


#endif
