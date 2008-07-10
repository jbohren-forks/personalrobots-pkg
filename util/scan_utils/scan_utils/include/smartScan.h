#ifndef _smartscan_h_
#define _smartscan_h_

#include <std_msgs/Point3DFloat32.h> //ROS native format for a point cloud
#include <dataTypes.h> //my own data types defined in this library; probably just placeholder
//until ROS gets similar types


#include <vector>
#include <iostream>
#include <assert.h>

class vtkPolyData;
class vtkPointLocator;
class vtkTransform;
class vtkTransformPolyDataFilter;
class vtkTransformFilter;

namespace NEWMAT {
	class Matrix;
}
namespace libTF {
	class TransformReference;
}

/*!
  This class holds most of the functionality of the Scan_Utils library.

  A SmartScan is essentially a point cloud which can apply a number of
  tools on itself (or other clouds). It holds the 3D vertex list which
  defines the cloud, but in order to apply various tools it might need
  more data or the same data in different formats. It will either
  create that for itself, or require it to be supplied.

  Intensity data is not stored or used in any way, for the
  moment. Serialization functions assume its existance to make it
  easier to load data saved by other tools, but it is not stored at
  all after it is read.
  
  The original point cloud (referred to here as the "native" data is
  held in a ROS compatible format. This is so that in the future, if a
  new cloud is received over the ROS message system, we can store it
  directly without requiring an extra copy. However, this
  functionality is not implemented for now.

  Since most tools provided by this library are from VTK, this class
  also holds a copy of the point cloud in VTK format. This includes both a
  list of vertices and a populated spatial search structure similar to
  a kd-tree. The external user does not need to know about the VTK
  version of the data; it is the job of all public functions of this
  class to ensure that the VTK data is created when needed and then
  kept up to date when the "native" data is modified.
 */
class SmartScan {
 private:
	//! "Native" data: an array of 3D vertices stored as ROS Point3DFloat32
	std_msgs::Point3DFloat32 *mNativePoints;
	//! The number of vertices in the cloud
	int mNumPoints;
	//! A transform associated with this point cloud
	libTF::TransformReference *mTransform;

	//! Copy of the 3D cloud in VTK format
	vtkPolyData *mVtkNativeData, *mVtkData;
	//! Spatial structure similar to a kd-tree that VTK uses for fast neighbor searches in the cloud
	vtkPointLocator *mVtkPointLocator;
	//! Copy of the inner transform in VTK format
	vtkTransform *mVtkTransform;
	//! VTK Filter for applying the VTK transform to the VTK Data
	vtkTransformFilter *mVtkTransformFilter;
	//! Copies the native data into a VTK-compatible structure
	void createVtkData();
	//! Sets the VTK version of the transform to match the libTF version
	void setVtkTransform();
	//! Deletes the VTK version of the native data
	void deleteVtkData();
	//! Returns true if the VTK version of the native data exists and the class is ready to use VTK-based tools
	bool hasVtkData(){return (mVtkData!=NULL);}
	//! Returns a pointer to the VTK version of the point cloud
	vtkPolyData *getVtkData();
	//! Returns a pointer to the VTK spatial search structure
	vtkPointLocator *getVtkLocator();

	//! Clears all the data held by this class and frees all memory footprint. Does not change inner transform.
	void clearData();
	//! Sets the inner transform to identity
	void clearTransform();

	//! Apply the inner transform to a point
	std_msgs::Point3DFloat32 transformPoint(const std_msgs::Point3DFloat32 &p) const;
	//! Fit plane to normalized points using SVD and return normal direction.
	std_msgs::Point3DFloat32 SVDPlaneNormal(NEWMAT::Matrix *M, int n);
 public:
	//! Empty constructor for a new point cloud that does not hold any data
	SmartScan();
	~SmartScan();

	//! Set the native data of the point cloud
	void setPoints(int numPoints, const std_msgs::Point3DFloat32 *points);

	//! Write point cloud to an output stream
	void writeToFile(std::iostream &output);
	//! Read point cloud from an input stream. 
	bool readFromFile(std::iostream &input);
	//! Returns the number of points in the cloud
	int size() const {return mNumPoints;}
	//! Returns the i-th point in the cloud.
	std_msgs::Point3DFloat32 getPoint(int i) const;


	//! Set the inner transform using a NEWMAT matrix
	void setTransform(const NEWMAT::Matrix &M);
	//! Set the inner transform using a row-major float[16]
	void setTransform(float *t);
	//! Get the inner transform in a NEWMAT matrix
	void getTransform(NEWMAT::Matrix &M);
	//! Get the inner transform in a row-major float[16]
	void getTransform(float *t);
	//! Apply a transform on top to the existing inner transform
	void applyTransform(NEWMAT::Matrix &M);
	//! Apply a transform on top to the existing inner transform
	void applyTransform(float *t);

	//! Performs 2D Delaunay Triangulation on the point cloud.
	std::vector<scan_utils::Triangle> *delaunayTriangulation(double tolerance, double alpha);
	//! Performs 3D Delaunay Traingulation on the point cloud.
	std::vector<scan_utils::Triangle> *delaunayTriangulation3D(double tolerance, double alpha);
	//! Crops the point cloud to a 3D bounding box
	void crop(float x, float y, float z, float dx, float dy, float dz);
	//! Removes outliers - points that have few neighbors
	void removeOutliers(float radius, int nbrs);
	//! Removes points whose normals are perpendicular to the direction of the scanner
	void removeGrazingPoints(float threshold, bool removeOutliers = true, float radius = 0.01, int nbrs = 5);
	//! Registers this point cloud using ICP to another scan.
	float* ICPTo(SmartScan *target);
	//! Finds the dominant plane in the point cloud by histograming point normals
	void normalHistogramPlane(std_msgs::Point3DFloat32 &planePoint, std_msgs::Point3DFloat32 &planeNormal,
				  float radius = 0.01, int nbrs = 5);
	//! Finds the dominant plane in the point cloud using RANSAC
	void ransacPlane(std_msgs::Point3DFloat32 &planePoint, std_msgs::Point3DFloat32 &planeNormal,
			 int iterations = 500, float distThresh = 0.02);
	//! Removes all points that are close to a given plane
	void removePlane(const std_msgs::Point3DFloat32 &planePoint, 
			 const std_msgs::Point3DFloat32 &planeNormal, float thresh = 0.02);
	//! Computes the normal of a point by looking at its neighbors
	std_msgs::Point3DFloat32 computePointNormal(int id, float radius = 0.01, int nbrs = 5);
};


#endif
