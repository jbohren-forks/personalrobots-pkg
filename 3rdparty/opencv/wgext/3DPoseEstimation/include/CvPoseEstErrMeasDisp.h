/*
 * CvPoseEstErrMeasDisp.h
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#ifndef CVPOSEESTERRMEASDISP_H_
#define CVPOSEESTERRMEASDISP_H_

#include "CvPoseEstErrMeas.h"
#include "CvStereoCamModel.h"

class CvPoseEstErrMeasDisp: public CvPoseEstErrMeas, public CvStereoCamModel {
public:
	CvPoseEstErrMeasDisp();
	virtual ~CvPoseEstErrMeasDisp();
	void transform(const CvMat& src, CvMat& dst);
	void measure(const CvMat& uvds0, const CvMat& uvds1);
	void measureMixed(CvMat& xyzs0, CvMat& uvds1);
};

#endif /* CVPOSEESTERRMEASDISP_H_ */
