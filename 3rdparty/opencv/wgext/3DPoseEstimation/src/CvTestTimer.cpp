#include "CvTestTimer.h"
#include <cxcore.h>

#include <stdio.h>
#include <iostream>

CvTestTimer CvTestTimer::_singleton;

CvTestTimer::CvTestTimer():
	mFrequency(cvGetTickFrequency())
{
	reset();
}

CvTestTimer::~CvTestTimer()
{
}

#define PRINTSTAT(title, name) printStat((title), m##name, mCount##name)

void CvTestTimer::printStat(const char* title, int64 val, int64 count) {
	fprintf(stdout, "total time spend in %s: %10.2f, %6.2f%%, %10.2f\n",
			title,
			(double)val/(double)mNumIters/(double)mFrequency, 
			(double)val/(double)mTotal*100.0, (double)count/(double)mNumIters);
}

void CvTestTimer::printStat() {
	cout << "num of iters: "<< this->mNumIters<<endl;
	PRINTSTAT("Total               ", Total);
	PRINTSTAT("SVD                 ", SVD);
	PRINTSTAT("CheckInliers        ", CheckInliers);
	PRINTSTAT("CopyInliers         ", CopyInliers);
	PRINTSTAT("LevMarq::           ", LevMarqDoit);
	PRINTSTAT("LevMarq::getResidue ", Residue);
	PRINTSTAT("LevMarq::errNorm    ", ErrNorm);
	PRINTSTAT("LevMarq::JtJJtErr   ", JtJJtErr);
	PRINTSTAT("LevMarq::CvLevMarq  ", CvLevMarq);
	PRINTSTAT("LevMarq::CnstrctMats", ConstructMatrices);
}

