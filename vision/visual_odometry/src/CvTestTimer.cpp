#include "CvTestTimer.h"

#include <stdio.h>
#include <iostream>
#include <opencv/cxcore.h>


CvTestTimer CvTestTimer::_singleton;

CvTestTimer::CvTestTimer():
  mFrequency(CLOCKS_PER_SEC/1000)  // make it milli seconds. Go with clock()
//	mFrequency(cvGetTickFrequency()*1000)  // make it milli seconds. cvGetTickFrequency()
	// returns the number of ticks in one micro second.
{
	reset();
}

CvTestTimer::~CvTestTimer()
{
}

// Depreciated
#define PRINTSTAT(title, name) do {printStat((title), m##name, mCount##name);}while (0)

#define PRINTSTAT2(title, name) do {printStat((title), m##name.mTime, m##name.mCount);} while(0)

#define PRINTSTATSBA(title, name) do {printStatSBA((title), m##name.mTime, m##name.mCount);} while(0)

void CvTestTimer::printStat(const char* title, int64 val, int64 count) {
  fprintf(stdout, "%s: %10.2f, %6.2f%%, %10.2f\n",
      title,
      (mNumIters>0&&mFrequency>0)?((double)val/(double)mNumIters/(double)mFrequency):0.0,
      (mTotal>0&&mNumIters>0)?    ((double)val/(double)mTotal*100.0)      :0.0,
      (mNumIters>0)?              ((double)count/(double)mNumIters)       :0.0);
}

void CvTestTimer::printStatSBA(const char* title, int64 val, int64 count) {
  fprintf(stdout, "%s: %11.3f, %7.3f%%, %11.3f, %11.3f, %13.3f%%\n",
      title,
      // average time
      (mNumIters>0&&mFrequency>0)?((double)val/(double)mNumIters/(double)mFrequency):0.0,
      // percentage of the total time
      (mTotal>0&&mNumIters>0)?    ((double)val/(double)mTotal*100.0)      :0.0,
      // average frequency
      (mNumIters>0)?              ((double)count/(double)mNumIters)       :0.0,
      // average time in each sparse bundle adjustment iteration
      ((double)val/(double)mSparseBundleAdj.mCount/(double)mFrequency),
      // percentage of the total time in sparse bundle adjustment
      ((double)val/(double)mSparseBundleAdj.mTime*100.0)
  );
}

void CvTestTimer::printStat() {
  cout << "Statistics of all counters (time in milliseconds, i.e .001 seconds)"<<endl;
  cout << "num of iters: "<< mNumIters<<endl;
  if (mNumIters==0) {
    cerr << "Please set mNumIters, the number of iterations"<<endl;
    return;
  }
  cout <<      "[counter]              [Avg time] [% of Total] [Avg Freq]"<<endl;
	PRINTSTAT   ("Total                ", Total);
#if 0
	PRINTSTAT   ("LevMarq::            ", LevMarqDoit);
	PRINTSTAT2  ("LevMarq::DoitAlt2    ", LevMarq2);
	PRINTSTAT2  ("LevMarq::DoitAlt3    ", LevMarq3);
	PRINTSTAT2  ("LevMarq::DoitAlt4    ", LevMarq4);
	PRINTSTAT2  ("LevMarq::DoitAlt5    ", LevMarq5);
	PRINTSTAT2  ("LevMarq::getResidue  ", Residue);
	PRINTSTAT2  ("LevMarq::FwdResidue  ", FwdResidue);
	PRINTSTAT   ("LevMarq::errNorm     ", ErrNorm);
	PRINTSTAT   ("LevMarq::JtJJtErr    ", JtJJtErr);
	PRINTSTAT   ("LevMarq::LevMarq     ", LevMarq);
	PRINTSTAT   ("LevMarq::CnstrctMats ", ConstructMatrices);
	PRINTSTAT2  ("LoadImage            ", LoadImage);
	PRINTSTAT2  ("DisparityMap         ", DisparityMap);
#endif
	PRINTSTAT2  ("FeaturePoint         ", FeaturePoint);
	PRINTSTAT2  ("KeypointDescriptor   ", KeyPointDescriptor);
  PRINTSTAT2  ("KeypointMatch        ", KeyPointMatch);
  PRINTSTAT2  ("SparseStereo         ", SparseStereo);
	PRINTSTAT2  ("PoseEstimateRANSAC   ", PoseEstimateRANSAC);
	PRINTSTAT2  ("  PointPicking       ", PointPicking);
	PRINTSTAT2  ("    RandTripletGen   ", RandTripletGenerator);
	PRINTSTAT2  ("    ColinearCheck    ", ColinearCheck);
  PRINTSTAT2  ("  SVD                ", SVD);
  PRINTSTAT   ("  CheckInliers       ", CheckInliers);
  PRINTSTAT   ("  CopyInliers        ", CopyInliers);
	PRINTSTAT2  ("PoseEstimateLevMarq  ", PoseEstimateLevMarq);
  cout <<      "[counter]              [Avg time] [% of Total] [Avg Freq] [SBA Avg time] [% of SBA Total]"<<endl;
	PRINTSTATSBA("SparseBundleAdjust   ", SparseBundleAdj);
	PRINTSTATSBA("SBA::CostFunction    ", SBACostFunction);
  PRINTSTATSBA("SBA::Derivatives     ", SBADerivatives);
  PRINTSTATSBA("SBA::DerivativesHpp  ", SBADerivativesHpp);
  PRINTSTATSBA("SBA::DerivHppInv     ", SBADerivativesHppInv);
  PRINTSTATSBA("SBA::DerivativesJc   ", SBADerivativesJc);
  PRINTSTATSBA("SBA::DerivHccHpc     ", SBADerivativesHccHpc);
	PRINTSTATSBA("SBA::FwdTransfMats   ", SBAFwdTransfMats);
	PRINTSTATSBA("SBA::OuterProdOfTrack", SBAOuterProdOfTrack);
	PRINTSTATSBA("SBA::LinearSolving   ", SBALinearSolving);
	PRINTSTATSBA("SBA::BackSubstitution", SBABackSubstitution);
}

