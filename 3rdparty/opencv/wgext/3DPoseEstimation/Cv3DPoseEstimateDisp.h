#ifndef CV3DPOSEESTIMATEDISP_H_
#define CV3DPOSEESTIMATEDISP_H_

#include "Cv3DPoseEstimateDispSpaceRef.h"

class Cv3DPoseEstimateDisp : public Cv3DPoseEstimateDispSpaceRef
{
public:
	Cv3DPoseEstimateDisp();
	virtual ~Cv3DPoseEstimateDisp();
protected:
	virtual int checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation);
	virtual int getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
	    CvMat* points0Inlier, CvMat* points1Inlier);
};

#endif /*CV3DPOSEESTIMATEDISP_H_*/
