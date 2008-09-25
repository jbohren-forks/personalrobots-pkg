#include "cv.h"

int main()
{
  CvMat* mat = cvCreateMat( 5, 5, CV_32FC1 );
  float element_3_2 = 7.7;
  *( (float*)CV_MAT_ELEM_PTR( *mat, 3, 2 ) ) = element_3_2;

  // below from example 3-8+
  cvmSet( mat, 2, 2, 0.5000 );
  cvSetReal2D( mat, 2, 2, 0.5000 );
}
