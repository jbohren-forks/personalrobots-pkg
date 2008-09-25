#include "cv.h"

int main()
{
  CvMat* mat = cvCreateMat( 5, 5, CV_32FC1 );
  float element_3_2 = CV_MAT_ELEM( *mat, float, 3, 2 );
}
