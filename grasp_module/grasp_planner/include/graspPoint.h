#ifndef _grasppoint_h_
#define _grasppoint_h_

#include <libTF/libTF.h>

namespace NEWMAT {
	class Matrix;
}

namespace grasp_module {

/*!  A GraspPoint stores the information needed to grasp an object at
  a certain location.

  For the moment, we are storing this information in the form of a
  transformation matrix that aligns the gripper with the desired
  grasping location. 

  COORDINATE SYSTEM CONVENTION: here we assume that the GRIPPER
  coordinate system is as follows:

  - origin is in the middle of the empty space between the jaws

  - x axis is pointing forward (towards the object to be grasped)

  - y axis is along the axis of translation of the parallel jaws

  - z axis is the cross product of the two

  If for example grasping a TALL VERTICAL object such as a bottle,
  with the jaws on each side of it, we would have the x axis pointing
  towards the object, the y axis pointing to the left and the z axis
  pointing up. The origin would be somewhere inside the bottle,
  presumably at its centroid.

  If this is not what the gripper coordinate system should be, this
  class as well as the grasp planner need to be modified to reflect
  that.
  
  In order to perform a grasp when starting from an instance of the
  GraspPoint, the following steps have to be performed:

  1. Place the gripper at the 3D location and orientation specified by
  the GraspPoint.

  2. Close the gripper.

  3. Hope for the best.

 */
class GraspPoint {
 private:
	//! The inner grasp point transform
	libTF::TransformReference mTran;
	//! The quality value, between 0 and 1. Higher value means better grasp.
	float mQuality;
 public:
	static const unsigned int baseFrame;
	static const unsigned int graspFrame;
	static const unsigned int newFrame;

	//! Empty constructor initializes GraspPoint to identity and quality to 0
	GraspPoint();
	//! Constructor taking all parameters
	GraspPoint(const libTF::TFPoint &c, 
		   const libTF::TFVector &app, const libTF::TFVector &up) {
		setTran(c,app,up);
	}
	GraspPoint(const NEWMAT::Matrix *M) {setTran(M);}

	//! Set the grasp point transform using a 4x4 NEWMAT matrix
	void setTran(const NEWMAT::Matrix *M);
	//! Set the transform by specifying position and approach (x) and up (z) directions 
	void setTran(const libTF::TFPoint &c, 
		     const libTF::TFVector &app, const libTF::TFVector &up);

	//! Returns the inner transform as a row-major float[16]. Caller should free this memory
	void getTran(float **f);
	//! Returns the inner transform as a NEWMAT::Matrix
	void getTran(NEWMAT::Matrix *M);

	//! Set the quality of this grasp. Caps value between 0 and 1
	void setQuality(float q){if (q<0) q=0; if (q>1) q=1; mQuality = q;}
	//! Get the quality of this grasp.
	float getQuality() const {return mQuality;}
	//! For sorting based on relative quality values.
	bool operator < (const GraspPoint &rhs) const {return mQuality < rhs.mQuality;}
};
 
//!Convenience predicate for sorting containers of pointers to GraspPoints
 bool sortGraspPredicate(const GraspPoint *lhs, const GraspPoint *rhs);

//a couple of conveniece fctns
 float norm(const libTF::TFVector &f);
 libTF::TFVector normalize(const libTF::TFVector &f);
 float dot(const libTF::TFVector &f1, const libTF::TFVector &f2);
 libTF::TFVector cross(const libTF::TFVector &f1, const libTF::TFVector &f2);


} //namespace grasp_module

#endif
