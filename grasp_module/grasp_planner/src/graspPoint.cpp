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

void GraspPoint::getTran(float **f) const
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

void GraspPoint::getTran(NEWMAT::Matrix *M) const
{
	//get matrix is not const...
	*M = (const_cast<libTF::TransformReference&>(mTran)).getMatrix(baseFrame,graspFrame, 0);
}

graspPoint_msg GraspPoint::getMsg() const
{
	graspPoint_msg msg;
	NEWMAT::Matrix M(4,4);
	getTran(&M);

	msg.frame.xt = M.element(0,3);
	msg.frame.yt = M.element(1,3);
	msg.frame.zt = M.element(2,3);
	msg.quality = getQuality();

	//my own matrix-to-quaternion conversion for now...

	// Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
	// article "Quaternion Calculus and Fast Animation".
	
	double trace = M.element(0,0)+M.element(1,1)+M.element(2,2);
	double root;
	
	if ( trace > 0.0 ){
		// |w| > 1/2, may as well choose w > 1/2
		root = sqrt(trace+1.0);  // 2w
		msg.frame.w = 0.5*root;
		root = 0.5/root;  // 1/(4w)
		msg.frame.xr = (-M.element(1,2)+M.element(2,1))*root;
		msg.frame.yr = (-M.element(2,0)+M.element(0,2))*root;
		msg.frame.zr = (-M.element(0,1)+M.element(1,0))*root;
	}
	else {
		// |w| <= 1/2
		static int next[3] = { 1, 2, 0 };
		int i = 0;
		if ( M.element(1,1) > M.element(0,0) )
			i = 1;
		if ( M.element(2,2) > M.element(i,i) )
			i = 2;
		int j = next[i];
		int k = next[j];
		
		root = sqrt(M.element(i,i)-M.element(j,j)-M.element(k,k)+1.0);
		double* quat[3] = { &msg.frame.xr, &msg.frame.yr, &msg.frame.zr };
		*quat[i] = 0.5*root;
		root = 0.5/root;
		msg.frame.w = (-M.element(j,k)+M.element(k,j))*root;
		*quat[j] = (M.element(i,j)+M.element(j,i))*root;
		*quat[k] = (M.element(i,k)+M.element(k,i))*root;
	}

	//normalise the quaternion
	double nm = msg.frame.xr * msg.frame.xr;
	nm += msg.frame.yr * msg.frame.yr;
	nm += msg.frame.zr * msg.frame.zr;
	nm += msg.frame.w  * msg.frame.w;

	double invnorm;
	if (nm==0.0) {
		//this should not happen
		invnorm = 0;
	} else {
		invnorm =1.0/sqrt(nm);
	}
	msg.frame.w*=invnorm; msg.frame.xr*=invnorm; 
	msg.frame.yr*=invnorm; msg.frame.zr*=invnorm;

	return msg;
}


void GraspPoint::setFromMsg(const graspPoint_msg &msg)
{
	mTran.setWithQuaternion( graspFrame, baseFrame, 
				 msg.frame.xt, msg.frame.yt, msg.frame.zt, 
				 msg.frame.xr, msg.frame.yr, msg.frame.zr, msg.frame.w,
				 0);
	setQuality(msg.quality);
}


} //namespace grasp_module
