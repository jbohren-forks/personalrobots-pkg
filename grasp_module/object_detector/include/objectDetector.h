#ifndef _objectdetector_h_
#define _objectdetector_h_

/**
   @mainpage 
   @b A simple object detector that cleans up point clouds
   and then segments into connected components. See ObjectDetector
   class for details.
 **/

#include <vector>

class SmartScan;

namespace grasp_module {

class Object;

/*!
  A simple detector for objects inside point clouds.

  The main function of this class is \a getObjects(...) which takes in
  a point cloud and returns the Objects found inside. However, this
  functionality needs a set of parameters. The empty constructor will
  initialize all these parameters to some default value, but it is
  recommended to study the parameters and decide what are the optimal
  values for the desired application.

  The \a getObjects(...) function also makes a series of assumptions
  about that is in the point cloud. In this sense, this is a very
  naive object detector, more of a stub for better things to come. See
  \a getObjects(...) function for all details.
 */
class ObjectDetector {
 private:
	//! Parameter for segmentation operation
	float mGrazThresh;
	//! Parameter for segmentation operation
	float mGrazRad;
	//! Parameter for segmentation operation
	int mGrazNbrs;
	//! Parameter for segmentation operation
	float mRemThresh;
	//! Parameter for segmentation operation
	float mFitThresh;
	//! Parameter for segmentation operation
	float mConnThresh;
	//! Parameter for segmentation operation
	int mSizeThresh;
	
 public:
	//! Constructor. Will initialize all unspecified parameters to a default value.
	ObjectDetector();
	//! Core function of this class, segments a point cloud into objects.
	std::vector<Object*> *getObjects(SmartScan *scan);

	//! Set parameters for removing grazing points
	void setParamGrazing(float thresh, float rad, int nbrs);
	//! Set paramentes for plane detection and removal
	void setParamPlane(float thresh, float fit);
	//! Set paramenters for connected components
	void setParamComponents(float thresh, int points);
};

} //namespace grasp_module

#endif
