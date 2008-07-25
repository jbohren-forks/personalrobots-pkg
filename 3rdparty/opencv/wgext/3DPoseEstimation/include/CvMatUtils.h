#ifndef CVMATUTILS_H_
#define CVMATUTILS_H_

#include <cxcore.h>

typedef double CvMyReal;
#define CV_XF CV_64F

class CvMatUtils
{
public:
	CvMatUtils();
	virtual ~CvMatUtils();
    static void printMat(const CvMat *mat, const char *format="%12.5f,");
};

#endif /*CVMATUTILS_H_*/
