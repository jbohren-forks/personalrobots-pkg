#include "objectDetector.h"

#include "smartScan.h"
#include "object.h"

namespace grasp_module {

/*! Empty constructor sets default values for all parameters. These
    should at least be a good starting point for most situations. See
    \a setParamXX(...) in this class for parameter details.
 */
ObjectDetector::ObjectDetector()
{
	mGrazThresh = 10;
	mGrazRad = 0.01;
	mGrazNbrs = 5;
	mRemThresh = 0.02;
	mFitThresh = 0.7;
	mConnThresh = 0.02;
	mSizeThresh = 300;
}

/*! This function returns the vector of \a Objects found in \a
 scan. It proceeds through the following steps:

 1. Removal of ghosting artifacts. We assume point cloud is coming
 from Hokuyo scanner, which produces artifacts. This step uses \a
 SmartScan::removeGrazingPoints(...) to eliminate these.

2. Table detection. We assume that a) the object all lie on a planar
surface such as a table; b) the scan has been pre-cropped so that the
table is the dominant plane in the scan (no walls, floors etc). This
step detects the dominant plane in the scan and then removes it.

3. Decompose scan into connected components. Once the table has been
removed, the connected components that are left should be the objects
we are interested in. Of course, we might still have stuff under the
table or way above - in future work we will also do more cropping here
based on the detected table.

4. Discard potential objects that do not have enough points to be
considered reliable.

5. Compute controids and principal axes for what's left, using \a
SmartScan::principalAxes(...)

One potential problem is that this does not leave the scan untouched
(it has to remove things). Maybe in the future we'll also write a
version that takes in a \a const \a SmartScan at the price of a couple
of extra copies.
  */

std::vector<Object*> *ObjectDetector::getObjects(SmartScan *scan)
{
	std::vector<Object*> *objects = new std::vector<Object*>;

	if (mGrazThresh != 0) {
		fprintf(stderr,"Removing grazing points...");
		//clean up grazing points and outliers
		scan->removeGrazingPoints(mGrazThresh, true, mGrazRad, mGrazNbrs); \
		fprintf(stderr," done.\n");
	}

	if (mRemThresh != 0) {
		//find table
		fprintf(stderr,"Finding table...");
		std_msgs::Point3DFloat32 planePoint, planeNormal;
		float fitValue = scan->ransacPlane(planePoint, planeNormal);
		fprintf(stderr," done.\n");

		//clean up table
		if (fitValue > mFitThresh) {
			fprintf(stderr,"Removing table...");
			scan->removePlane(planePoint, planeNormal, mRemThresh);
			fprintf(stderr," done.\n");
		} else {
			fprintf(stderr,"Plane fit under threshold, we are not removing it...\n");
		}
	}

	//remove outliers again
	fprintf(stderr,"Removing outliers...");
	scan->removeOutliers(mGrazRad, mGrazNbrs);
	fprintf(stderr," done.\n");

	//get connected components
	fprintf(stderr,"Computing connected components...");
	std::vector<SmartScan*> *components = scan->connectedComponents(mConnThresh);
	fprintf(stderr," done.\n");

	libTF::TFPoint c;
	libTF::TFVector a1,a2,a3;
	SmartScan *comp;
	fprintf(stderr,"Creating objects...");
	while (!components->empty()) {
		comp = components->back();
		components->pop_back();

		//discard low-volume components
		if (comp->size() > mSizeThresh) {
			//compute centroid and principal axes
			c = comp->centroid();
			comp->principalAxes(a1,a2,a3);

			//create objects
			objects->push_back( new Object(c,a1,a2,a3) );
		}
		delete comp;
	}
	fprintf(stderr," done.\n");

	return objects;
}


/*! There parameters are passed to \a SmartScan::removeGrazingPoints(...)

   \param thresh - distance between point normal and scan
   perpendicular direction for removal, in degrees. Set to 0 if you
   want to completely skip the step of removing grazing
   points. Default = 10

   \param rad \param nbrs - radius and number of neighbors for
   computing normals. Defaults = 0.01 and 5

   See \a SmartScan::removeGrazingPoints(...) for details
 */
void ObjectDetector::setParamGrazing(float thresh, float rad, int nbrs)
{
	mGrazThresh = thresh;
	mGrazRad = rad;
	mGrazNbrs = nbrs;
}

/*! These parameters are passed to \a SmartScan::removePlane(...)

   \param thresh - points that are closer than this value to the
   detected plane of the table are removed. Set to 0 if you want the
   plane detection and removal skipped entirely. Default = 0.02

   \param fit - if the plane contains less than \a fit * total points
   in the scan, we consider that the scan actually does not have a
   dominant plane (such as a table) and therefore we skip the plane
   removal stage. This value is directly compared to the return value of
   \a SmartScan::ransacPlane(...). Default = 0.7
 */
void ObjectDetector::setParamPlane(float thresh, float fit)
{
	mRemThresh = thresh;
	mFitThresh = fit;
}

/*! These parameters are passed to \a SmartScan::connectedComponents(...)

  \param thresh - two components are considered connected if they are
  closer than this value. Default = 0.02

  \param points - only components with more point than this value are
  kept. Default = 300

 */
void ObjectDetector::setParamComponents(float thresh, int points)
{
	mConnThresh = thresh;
	mSizeThresh = points;
}

} //namespace grasp_module
