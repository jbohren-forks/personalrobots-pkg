#ifndef _object_h_
#define _object_h_

#include "assert.h"

#include <libTF/libTF.h>

namespace grasp_module {

/*!  This class represents an abstraction of an object, as opposed to
  a set of raw sensor data. It is designed to be an intermediate step
  between the raw data and the grasp planner: the raw data is
  segmented into instances of this class, which are in turn passed to
  the grasp planner.

  For now, this can be considered a stub, holding a very simple
  implementation. It is designed to work with a simple segmentation of
  connected componetns from point clouds. It holds the centroid and
  the principal axes of the object as detected in the point cloud.

  We are working for now with the principle that we only add more data
  if we need it. In the future, we might therefore add more labels,
  data points, etc. as needed.
 */

class Object {
 private:
	//! Object centroid
	libTF::TFPoint mCentroid;
	//! Object principal axes, ordered from most dominant (\a mA1 ) to least dominant (\a mA3 )
	libTF::TFVector mA1, mA2, mA3;

	//! Copy another object
	void copyFrom(const Object &o) {
		mCentroid = o.mCentroid;
		mA1 = o.mA1; mA2 = o.mA2; mA3 = o.mA3; }

 public:
	//! Constructor using centroid and axes information
	Object(const libTF::TFPoint &c, const libTF::TFVector &a1,
	       const libTF::TFVector &a2, const libTF::TFVector &a3) {
		setCentroid(c);
		setAxes(a1,a2,a3);
	}
	//! Copy constructor
	Object(const Object &o){copyFrom(o);}
	//! Assignment operator
	Object& operator=(const Object &o){copyFrom(o); return *this;}

	//! Set the centroid of the object
	void setCentroid(const libTF::TFPoint &c){mCentroid = c;}
	//! Set the principal axes of the object, from most dominant (\a a1 ) to least dominant ( \a a3 )
	void setAxes(const libTF::TFVector &a1,
		     const libTF::TFVector &a2,
		     const libTF::TFVector &a3){mA1 = a1; mA2 = a2; mA3 = a3;}
	//! Set the centroid of this object
	libTF::TFPoint getCentroid() const {return mCentroid;}
	//! Get the principal axes of the object, from most dominant (\a a1 ) to least dominant ( \a a3 )
	void getAxes(libTF::TFVector &a1,
		     libTF::TFVector &a2,
		     libTF::TFVector &a3) const {a1 = mA1; a2 = mA2; a3 = mA3;}
	//! Get one principal axis. Legal values of \a i are between 1 (most dominant axis) and 3 (least dominant axis)
	libTF::TFVector getAxis(int i) const { 
		switch(i) {
		case 1: return mA1; break;
		case 2: return mA2; break;
		case 3: return mA3; break;
		default: assert(0);} 
	}

};

} //namespace grasp_module

#endif
