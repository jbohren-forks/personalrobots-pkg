#ifndef CV3DPOSEESTIMATESSE_H_
#define CV3DPOSEESTIMATESSE_H_

#include "Cv3DPoseEstimateRef.h"

namespace cv { namespace willow {
/**
 * Pose estimation optimized by SSE (not implemented yet)
 */
class Cv3DPoseEstimateSSE : public PoseEstimate
{
public:
	Cv3DPoseEstimateSSE();
	virtual ~Cv3DPoseEstimateSSE();
};
}
}
#endif /*CV3DPOSEESTIMATESSE_H_*/
