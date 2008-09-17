#include "Cv3DPoseEstimateDisp.h"

#include "CvMat3X3.h"
#include "CvLevMarq3D.h"

#include <iostream>
#include "CvTestTimer.h"
using namespace std;

#undef DEBUG
#define USE_LEVMARQ
//#define DEBUG_DISTURB_PARAM

Cv3DPoseEstimateDisp::Cv3DPoseEstimateDisp()
{
}

Cv3DPoseEstimateDisp::~Cv3DPoseEstimateDisp()
{
}

int Cv3DPoseEstimateDisp::checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation){
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		CvMyReal p0x, p0y, p0z;

#if 1
		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
#else
		// not worth using the following code
		// 1) not sure if it helps on speed.
		// 2) not sure if the order of evaluation is preserved as left to right.
		CvMyReal w3 =
			_T[12]*(p0x=*_P0++) +
			_T[13]*(p0y=*_P0++) +
			_T[14]*(p0z=*_P0++) +
			_T[15];
#endif

		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

		numInLiers++;
	}
#ifdef DEBUG
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;
}

// almost the same as the function above
int Cv3DPoseEstimateDisp::getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
    CvMat* points0Inlier, CvMat* points1Inlier) {
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		CvMyReal p0x, p0y, p0z;

		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

        // store the inlier
        if (points0Inlier) {
            cvmSet(points0Inlier, numInLiers, 0, p0x);
            cvmSet(points0Inlier, numInLiers, 1, p0y);
            cvmSet(points0Inlier, numInLiers, 2, p0z);
        }
        if (points1Inlier) {
            cvmSet(points1Inlier, numInLiers, 0, *(_P1-3));
            cvmSet(points1Inlier, numInLiers, 1, *(_P1-2));
            cvmSet(points1Inlier, numInLiers, 2, *(_P1-1));
        }
		numInLiers++;
	}
#ifdef DEBUG
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;
}

int Cv3DPoseEstimateDisp::estimate(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
    CvMat& rot, CvMat& shift, bool reversed) {
  int numTrackablePairs = trackablePairs.size();
  double _uvds0[3*numTrackablePairs];
  double _uvds1[3*numTrackablePairs];
  CvMat uvds0 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds0);
  CvMat uvds1 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds1);

  int iPt=0;
  for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = trackablePairs.begin();
  iter != trackablePairs.end(); iter++) {
    const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
    _uvds0[iPt*3 + 0] = p.first.x;
    _uvds0[iPt*3 + 1] = p.first.y;
    _uvds0[iPt*3 + 2] = p.first.z;

    _uvds1[iPt*3 + 0] = p.second.x;
    _uvds1[iPt*3 + 1] = p.second.y;
    _uvds1[iPt*3 + 2] = p.second.z;
    iPt++;
  }

  // estimate the transform the observed points from current back to last position
  // it should be equivalent to the transform of the camera frame from
  // last position to current position
  int numInliers;
  if (reversed == true) {
    numInliers = this->estimate(&uvds1, &uvds0, &rot, &shift);
  } else {
    numInliers = this->estimate(&uvds0, &uvds1, &rot, &shift);
  }
  return numInliers;
}


