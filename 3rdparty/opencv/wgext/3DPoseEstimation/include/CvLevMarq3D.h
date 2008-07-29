#ifndef CVLEVMARQ3D_H_
#define CVLEVMARQ3D_H_

#include "calib_stereo.h"
#include "CvMatUtils.h"

//class CvLevMarq3D: public CvLevMarq
class CvLevMarqTransform
{
public:
	CvLevMarqTransform(int numErrors, int numMaxIter = defNumMaxIter);
	virtual ~CvLevMarqTransform();
	const static int numParams = 6;
	const static int defNumMaxIter = 50;  // maximum num of iterations
	const static int defMaxTimesOfUpdates = 300; //maximum num of times update() is called

	/**
	 *  initParams [alpha, beta, gamma, x, y, z];
	 */
	bool optimize(CvMat* A, CvMat* B, double param[]=NULL);
	bool optimize(CvMat* A, CvMat* B, CvMat *rot, CvMat* trans);
	
	typedef enum {
		Euler, Rodrigues
	} AngleType;
	AngleType mAngleType;
	
protected:
	bool constructRTMatrix(const CvMat* param);
	bool constructRTMatrices(const CvMat* param, CvMyReal delta);
	bool constructRTMatrix(const CvMat * param, CvMyReal _RT[]);
	bool computeForwardResidues(CvMat *xyzs0, CvMat *xyzs1, CvMat *res);
	virtual bool constructTransformationMatrix(const CvMat *param);
	virtual bool constructTransformationMatrix(const CvMat *param, CvMyReal T[]);
	virtual bool constructTransformationMatrices(const CvMat *param, CvMyReal delta);
//	virtual bool computeResidueVector(CvMat *xyzws0, CvMat *xyzws1, CvMat* resVector);
	virtual bool computeResidue(CvMat *xyzs0, CvMat *xyzs1, CvMat* res);
	// TODO: what is T supposed to be, 4x3 or 4x4?
	virtual bool computeResidue(CvMat *xyzs0, CvMat *xyzs1, CvMat* T, CvMat* res);
	// use CvLevMarq.updateAlt()
	bool optimizeAlt(const CvMat* A, const CvMat* B, 
			double param[]=NULL);
	// use CvLevMarq.update()
	bool optimizeDefault(CvMat* A, CvMat* B, double param[]=NULL);
	CvLevMarq mLevMarq;
	CvMyReal mRTData[16];
	CvMat mRT;  // this is a transient buffer. Do not assume it is updated
	CvMat mRT3x4;
	
	// buffer for transformation matrix of current param plus a delta vector
	// use in Jacobian approximation
	CvMyReal mFwdTData[numParams][16];
	CvMat mFwdT[numParams];
	// a view of mFwdT of the upper 3x4 of the matrix
	CvMat mFwdT3x4[numParams];
};

#endif /*CVLEVMARQ3D_H_*/
