#include <iostream.h>
#include "CvMatUtils.h"

CvMatUtils::CvMatUtils()
{
}

CvMatUtils::~CvMatUtils()
{
}

void CvMatUtils::printMat(const CvMat *mat, const char * format){
	cout << "A Matrix of "<<mat->rows<<" by "<< mat->cols <<endl;
	for (int i=0; i<mat->rows; i++) {
		for (int j=0; j<mat->cols; j++) {
			printf(format, cvmGet(mat, i, j));
		}
		cout << endl;
	}
}

