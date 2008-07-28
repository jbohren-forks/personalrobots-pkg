#ifndef WGLEVMARQDISPSPACE_H_
#define WGLEVMARQDISPSPACE_H_

#include <cxtypes.h>
#include "CvLevMarq3D.h"

class CvLevMarqDispSpace : public CvLevMarqTransform
{
public:
	CvLevMarqDispSpace(CvMat *disparityTo3D, CvMat *threeDToDisparity, int numErrors, 
        int numMaxInter = defNumMaxIter);
	virtual ~CvLevMarqDispSpace();
	/**
	 *  initParams [alpha, beta, gamma, x, y, z];
	 */
	bool doit(CvMat* A, CvMat* B, double param[]=NULL) {
		return doit1(A, B, param);
	}
private:
	bool constructHomographyMatrix(const CvMat* params);
	bool constructHomographyMatrix(const CvMat* params, CvMyReal H[]);
	virtual bool constructTransformationMatrix(const CvMat *param);
	virtual bool constructTransformationMatrix(const CvMat * param, CvMyReal T[]);
	virtual bool constructTransformationMatrices(const CvMat *param, CvMyReal delta);
	virtual bool computeResidueVector(const CvMat *xyzws0, const CvMat *xyzws1, CvMat* resVector);
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, CvMat* resVector);
	virtual bool computeResidue(const CvMat *xyzs0, const CvMat *xyzs1, const CvMat *T, CvMat* resVector);
	bool doit1(const CvMat *xyzs0, const CvMat *xyzs1, double _param[]);
	double _Homography[16];
	CvMat mHomography;
	
	CvMat *m3DToDisparity; // projection from 3D coordinate to disparity coordinates
	CvMat *mDisparityTo3D; // projection from disparity coordinate to 3D coordinates
};

#endif /*WGLEVMARQDISPSPACE_H_*/
