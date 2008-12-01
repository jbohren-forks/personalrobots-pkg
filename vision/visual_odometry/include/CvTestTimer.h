#include <opencv/cxtypes.h>
#include <opencv/cxcore.h>
#include <string>
using namespace std;
#ifndef CVTESTTIMER_H_
#define CVTESTTIMER_H_

#define DECLARE(timerName) RecordType m##timerName;
#define RESET(timerName)   do {m##timerName.reset();} while(0)

/**
 * setting up and displaying statistics of timers
 */
class CvTestTimer
{
public:
	CvTestTimer();
	virtual ~CvTestTimer();

	/// Timer record
	class RecordType {
	public:
		int64 mTime;
		int64 mCount;
		int64 mTimeStart;
		void reset() {
			mTime = 0;
			mCount = 0;
		}
	};

	void reset(){
	  mTotal   = 0;
		mCountTotal   = 0;
		mErrNorm = 0;
		mCountErrNorm = 0;
		mJtJJtErr = 0;
		mCountJtJJtErr = 0;
		mLevMarq  = 0;
		mCountLevMarq  = 0;
		mCvLevMarq2 = 0;
		mCountCvLevMarq2 = 0;
		mLevMarqDoit = 0;
		mCountLevMarqDoit = 0;
		mIsInLier = 0;
		mCountIsInLier = 0;
		mCheckInliers = 0;
		mCountCheckInliers = 0;
		mCopyInliers = 0;
		mCountCopyInliers = 0;
		mConstructMatrices = 0;
		mCountConstructMatrices = 0;
		RESET(SVD);
		RESET(Residue);
		RESET(FwdResidue);
		RESET(LevMarq2);
		RESET(LevMarq3);
		RESET(LevMarq4);
		RESET(LevMarq5);
    RESET(LoadImage);
    RESET(DisparityMap);
		RESET(FeaturePoint);
		RESET(PoseEstimateRANSAC);
		RESET(PoseEstimateLevMarq);
	  RESET(KeyPointMatch);
	  RESET(KeyPointDescriptor);
	  RESET(PointPicking);
	  RESET(ColinearCheck);
    RESET(RandTripletGenerator);
    RESET(SparseStereo);
    RESET(SparseBundleAdj);
    RESET(SBACostFunction);
    RESET(SBADerivatives);
    RESET(SBADerivativesHpp);
    RESET(SBADerivativesHppInv);
    RESET(SBADerivativesJc);
    RESET(SBADerivativesHccHpc);
    RESET(SBAFwdTransfMats);
    RESET(SBAOuterProdOfTrack);
    RESET(SBALinearSolving);
    RESET(SBABackSubstitution);
}
	int64 mNumIters;
	int64 mFrequency;

	int64 mTotal;
	int64 mCountTotal;

	int64 mErrNorm;
	int64 mCountErrNorm;
	int64 mJtJJtErr;
	int64 mCountJtJJtErr;
	int64 mLevMarq;
	int64 mCountLevMarq;
	int64 mCopyInliers;
	int64 mCountCopyInliers;
	int64 mCvLevMarq2;
	int64 mCountCvLevMarq2;
	int64 mLevMarqDoit;
	int64 mCountLevMarqDoit;
	int64 mIsInLier;
	int64 mCountIsInLier;
	int64 mCheckInliers;
	int64 mCountCheckInliers;
	int64 mConstructMatrices;
	int64 mCountConstructMatrices;

	DECLARE(SVD);
	DECLARE(Residue);
	DECLARE(FwdResidue);
	DECLARE(LevMarq2);
	DECLARE(LevMarq3);
	DECLARE(LevMarq4);
	DECLARE(LevMarq5);
	DECLARE(LoadImage);
	DECLARE(DisparityMap);
  DECLARE(FeaturePoint);
  DECLARE(PoseEstimateRANSAC);
  DECLARE(PoseEstimateLevMarq);
  DECLARE(KeyPointMatch);
  DECLARE(KeyPointDescriptor);
  DECLARE(PointPicking);
  DECLARE(ColinearCheck);
  DECLARE(RandTripletGenerator);
  DECLARE(SparseStereo);
  DECLARE(SparseBundleAdj);
  DECLARE(SBACostFunction);
  DECLARE(SBADerivatives);
  DECLARE(SBADerivativesHpp);
  DECLARE(SBADerivativesHppInv);
  DECLARE(SBADerivativesJc);
  DECLARE(SBADerivativesHccHpc);
  DECLARE(SBAFwdTransfMats);
  DECLARE(SBAOuterProdOfTrack);
  DECLARE(SBALinearSolving);
  DECLARE(SBABackSubstitution);

	void printStat();
  void printStat(const char* title, int64 val, int64 count);
  void printStatSBA(const char* title, int64 val, int64 count);
	static inline CvTestTimer& getTimer() {
		return _singleton;
	}
	static CvTestTimer _singleton;
};

//#define GETTICKCOUNT cvGetTickCount
#define GETTICKCOUNT clock

#define CvTestTimerStart(timerName) \
	{ int64 _CvTestTimer_##timerName = GETTICKCOUNT(); \
	  CvTestTimer::getTimer().mCount##timerName++;

#define CvTestTimerEnd(timerName) \
	CvTestTimer::getTimer().m##timerName += GETTICKCOUNT() - _CvTestTimer_##timerName;}

#define CvTestTimerStart2(timerName) \
	do { CvTestTimer::getTimer().m##timerName.mTimeStart = GETTICKCOUNT(); \
	  CvTestTimer::getTimer().m##timerName.mCount++;} while(0)

#define CvTestTimerEnd2(timerName) \
	do { CvTestTimer::getTimer().m##timerName.mTime += \
	  GETTICKCOUNT() - CvTestTimer::getTimer().m##timerName.mTimeStart;} while (0)

#endif /*CVTESTTIMER_H_*/
