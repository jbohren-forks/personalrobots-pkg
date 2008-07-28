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

#define PRINTSTAT(title, name) do {printStat((title), m##name, mCount##name);}while (0)

#define PRINTSTAT2(title, name) do {printStat((title), m##name.mTime, m##name.mCount);} while(0)

void CvTestTimer::printStat(const char* title, int64 val, int64 count) {
	fprintf(stdout, "total time spend in %s: %10.2f, %6.2f%%, %10.2f\n",
			title,
			(double)val/(double)mNumIters/(double)mFrequency, 
			(double)val/(double)mTotal*100.0, (double)count/(double)mNumIters);
}

void CvTestTimer::printStat() {
	cout << "num of iters: "<< this->mNumIters<<endl;
	PRINTSTAT("Total               ", Total);
	PRINTSTAT2("SVD                 ", SVD);
	PRINTSTAT("CheckInliers        ", CheckInliers);
	PRINTSTAT("CopyInliers         ", CopyInliers);
	PRINTSTAT("LevMarq::           ", LevMarqDoit);
	PRINTSTAT2("LevMarq::DoitAlt2   ", LevMarq2);
	PRINTSTAT2("LevMarq::DoitAlt3   ", LevMarq3);
	PRINTSTAT2("LevMarq::DoitAlt4   ", LevMarq4);
	PRINTSTAT2("LevMarq::DoitAlt5   ", LevMarq5);
	PRINTSTAT2("LevMarq::getResidue ", Residue);
	PRINTSTAT2("LevMarq::FwdResidue ", FwdResidue);
	PRINTSTAT("LevMarq::errNorm    ", ErrNorm);
	PRINTSTAT("LevMarq::JtJJtErr   ", JtJJtErr);
	PRINTSTAT("LevMarq::CvLevMarq  ", CvLevMarq);
	PRINTSTAT("LevMarq::CnstrctMats", ConstructMatrices);
}

