#include "graspPoint.h"

#include "newmat10/newmatap.h"

namespace grasp_module {

// A couple of convenience fctns until libTF gets this functionality as well
float norm(const libTF::TFVector &f)
{
	return sqrt( f.x*f.x + f.y*f.y + f.z*f.z);
}

libTF::TFVector normalize(const libTF::TFVector &f)
{
	float n = norm(f);
	libTF::TFVector p;
	p.x = f.x / n;
	p.y = f.y / n;
	p.z = f.z / n;
	return p;
}

float dot(const libTF::TFVector &f1, const libTF::TFVector &f2)
{
	return f1.x*f2.x + f1.y*f2.y + f1.z*f2.z;
}

libTF::TFVector cross(const libTF::TFVector &f1, const libTF::TFVector &f2)
{
	libTF::TFVector c;
	c.x = f1.y * f2.z - f1.z * f2.y;
	c.y = f1.z * f2.x - f1.x * f2.z;
	c.z = f1.x * f2.y - f1.y * f2.x;
	return c;
}

// Back to the GraspPoint

const unsigned int GraspPoint::baseFrame = 1;
const unsigned int GraspPoint::graspFrame = 2;
const unsigned int GraspPoint::newFrame = 3;

 bool sortGraspPredicate(const GraspPoint *lhs, const GraspPoint *rhs) {
	 return (*lhs) < (*rhs);
 }

GraspPoint::GraspPoint()
{
	NEWMAT::Matrix I(4,4);
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++) {
			if (i==j) I.element(i,j) = 1;
			else I.element(i,j) = 0;
		}
	}
	setTran(&I);
	mQuality = 0;
}

/*!  Sets the inner transform using a 4x4 NEWMAT matrix \a M. No
  checking is performed at all - the matrix is copied as is.
 */
void GraspPoint::setTran(const NEWMAT::Matrix *M)
{
	mTran.setWithMatrix( graspFrame, baseFrame, *M, 0);
}

/*! Sets the inner transform, but also normalizes everything and makes
    sure is forms a well-behaved right-handed coordinate system.

    \param c - the translation for the grasp point

    \param app - the approach direction, which is made to correspond
    to the x axis of the gripper. It is normalized, but other than
    that its direction is left untouched.

    \param up - the up direction, which is made to correspond to the z
    axis of the gripper. It is normalized, and if it is not
    perpendicular to \a app it is modified to make a correct
    coordinate system tranform.

 */
void GraspPoint::setTran(const libTF::TFPoint &c,
			 const libTF::TFVector &app, const libTF::TFVector &up)
{
	libTF::TFVector xaxis, yaxis, zaxis;

	//fprintf(stderr,"App: %f %f %f \n",app.x, app.y, app.z);
	//fprintf(stderr," Up: %f %f %f \n",up.x, up.y, up.z);


	xaxis = normalize(app);
	zaxis = normalize(up);
	yaxis = normalize( cross(zaxis,xaxis) );
	zaxis = cross(xaxis,yaxis);

	NEWMAT::Matrix M(4,4);

	M.element(0,3) = c.x;
	M.element(1,3) = c.y;
	M.element(2,3) = c.z;
	M.element(3,3) = 1.0;

	M.element(0,0) = xaxis.x;
	M.element(1,0) = xaxis.y;
	M.element(2,0) = xaxis.z;
	M.element(3,0) = 0.0;

	M.element(0,1) = yaxis.x;
	M.element(1,1) = yaxis.y;
	M.element(2,1) = yaxis.z;
	M.element(3,1) = 0.0;

	M.element(0,2) = zaxis.x;
	M.element(1,2) = zaxis.y;
	M.element(2,2) = zaxis.z;
	M.element(3,2) = 0.0;

	setTran(&M);

	/*
	std::cerr << "Matrix:" << std::endl << M;	
	NEWMAT::Matrix Q(4,4);
	getTran(&Q);
	std::cerr << "Output matrix:" << std::endl << Q;
	*/
}

void GraspPoint::getTran(float **f)
{
	*f = new float[16];
	NEWMAT::Matrix M(4,4);

	getTran(&M);
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++) {
			(*f)[4*i+j] = M.element(i,j);
		}
	}
}

void GraspPoint::getTran(NEWMAT::Matrix *M)
{
	//get matrix is not const...
	*M = mTran.getMatrix(baseFrame,graspFrame, 0);
}

} //namespace grasp_module
