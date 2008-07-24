#include <cxtypes.h>
#include <string>
using namespace std;
#ifndef CVTESTTIMER_H_
#define CVTESTTIMER_H_

class CvTestTimer
{
public:
	CvTestTimer();
	virtual ~CvTestTimer();
	
	void reset(){
		mResidue = 0;
		mTotal   = 0;
		mErrNorm = 0;
		mJtJJtErr = 0;
		mLevMarq  = 0;
		mCvLevMarq = 0;
		mLevMarqDoit = 0;
		mIsInLier = 0;
		mCheckInliers = 0;
	}
	int64 mNumIters;
	int64 mFrequency;
	int64 mTotal;
	
	int64 mSVD;
	int64 mResidue;
	int64 mErrNorm;
	int64 mJtJJtErr;
	int64 mLevMarq;
	int64 mCvLevMarq;
	int64 mLevMarqDoit;
	int64 mIsInLier;
	int64 mCheckInliers;
	void printStat();
	void printStat(const char* title, int64 val);
	static CvTestTimer& getTimer() {
		return _singleton;
	}
protected:
	static CvTestTimer _singleton;
};

#define CvTestTimerStart(timerName) \
	{ int64 _CvTestTimer_##timerName = cvGetTickCount();

#define CvTestTimerEnd(timerName) \
	CvTestTimer::getTimer().m##timerName += cvGetTickCount() - _CvTestTimer_##timerName;}


#endif /*CVTESTTIMER_H_*/
