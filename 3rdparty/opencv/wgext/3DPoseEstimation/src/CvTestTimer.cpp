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

void CvTestTimer::printStat(const char* title, int64 val) {
	fprintf(stdout, "total time spend in %s: %f, %f\n",
			title,
			(double)val/(double)mNumIters/(double)mFrequency, 
			(double)val/(double)mTotal);
}

void CvTestTimer::printStat() {
	cout << "num of iters: "<< this->mNumIters<<endl;
	printStat("Total", mTotal);
	printStat("SVD", mSVD);
	printStat("CheckInliers", mCheckInliers);
	
	printStat("LevMarq::doit", mLevMarqDoit);
	printStat("LevMarq::getResidue", mResidue);
	printStat("LevMarq::errNorm", mErrNorm);
	printStat("LevMarq::JtJJtErr", mJtJJtErr);
	printStat("LevMarq::CvLevMarq:", mCvLevMarq);
}

