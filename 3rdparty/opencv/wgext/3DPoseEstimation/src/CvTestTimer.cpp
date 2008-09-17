#include "CvTestTimer.h"

#include <stdio.h>
#include <iostream>
#include <opencv/cxcore.h>


CvTestTimer CvTestTimer::_singleton;

CvTestTimer::CvTestTimer():
	mFrequency(cvGetTickFrequency())
{
	reset();
}

CvTestTimer::~CvTestTimer()
{
}

// Depreciated
#define PRINTSTAT(title, name) do {printStat((title), m##name, mCount##name);}while (0)

#define PRINTSTAT2(title, name) do {printStat((title), m##name.mTime, m##name.mCount);} while(0)

void CvTestTimer::printStat(const char* title, int64 val, int64 count) {
	fprintf(stdout, "total time spend in %s: %10.2f, %6.2f%%, %10.2f\n",
	    title,
	    (mNumIters>0&&mFrequency>0)?((double)val/(double)mNumIters/(double)mFrequency):0.0,
	    (mTotal>0&&mNumIters>0)?    ((double)val/(double)mTotal*100.0)      :0.0,
	    (mNumIters>0)?              ((double)count/(double)mNumIters)       :0.0);
}

void CvTestTimer::printStat() {
  cout << "Statistics of all counters (time in milli seconds)"<<endl;
  cout << "[counter]  [Avg time] [% of Total] [Avg Freq]"<<endl;

	cout << "num of iters: "<< mNumIters<<endl;
	if (mNumIters==0) {
	  cerr << "Please set mNumIters, the number of iterations"<<endl;
	  return;
	}
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
	PRINTSTAT("LevMarq::CvLevMarq  ", CvLevMarq2);
	PRINTSTAT("LevMarq::CnstrctMats", ConstructMatrices);
	PRINTSTAT2("LoadImage           ", LoadImage);
	PRINTSTAT2("DisparityMap        ", DisparityMap);
	PRINTSTAT2("FeaturePoint        ", FeaturePoint);
	PRINTSTAT2("TrackablePair       ", TrackablePair);
	PRINTSTAT2("PoseEstimate        ", PoseEstimate);
}

