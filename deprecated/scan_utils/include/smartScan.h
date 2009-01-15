/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Matei Ciocarlie */

#ifndef _smartscan_h_
#define _smartscan_h_

#include <std_msgs/Point32.h> //ROS native format for a point cloud
#include <std_msgs/PointStamped.h> //ROS native format for a point cloud
#include <std_msgs/PointCloud.h>
#include <dataTypes.h> //my own data types defined in this library; probably just placeholder
//until ROS gets similar types

#include <libTF/libTF.h> //for transforms

#include <vector>
#include <iostream>
#include <assert.h>


class vtkPolyData;
class vtkPointLocator;
class vtkTransform;

namespace NEWMAT {
	class Matrix;
}

namespace scan_utils {
	template <typename T>
	class Octree;
}

//namespace libTF {
//	class TransformReference;
//}

/**
   @mainpage
   @b A library of tools for processing and manipulating point clouds. See SmartScan class for details.

   Also contains a templated Octree class for general use.
 **/

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
	//! "Native" data: an array of 3D vertices stored as ROS Point32
	std_msgs::Point32 *mNativePoints;
	//! The number of vertices in the cloud
	int mNumPoints;
	//! The location of the scanner itself. Defaults to (0,0,0).
	libTF::TFPoint mScannerPos;
	//! The orientation of the scanner. Defaults to look towards (1,0,0) with (0,0,1) pointing up
	libTF::TFVector mScannerDir, mScannerUp;

	//! Copy of the 3D cloud in VTK format
	vtkPolyData *mVtkData;
	//! Spatial structure similar to a kd-tree that VTK uses for fast neighbor searches in the cloud
	vtkPointLocator *mVtkPointLocator;
	//! Copies the native data into a VTK-compatible structure
	void createVtkData();
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

	//! Apply the inner transform to a point
	std_msgs::Point32 transformPoint(const std_msgs::Point32 &p,
						libTF::TransformReference &tr) const;
	//! Return the singular vectors of a Newmat matrix sorted in decreasing order of singular values
	void singularVectors(NEWMAT::Matrix *M, int n, std_msgs::Point32 &sv1, 
			     std_msgs::Point32 &sv2, std_msgs::Point32 &sv3);
 public:
	//! Empty constructor for a new point cloud that does not hold any data
	SmartScan();
	//! Destructor
	~SmartScan();

	//! Set the native data of the point cloud
	void setPoints(int numPoints, const std_msgs::Point32 *points);
	//! Set the native data as an array of floats; no ROS data type needed
	void setPoints(int numPoints, const float *points);
	//! Set scanner position and orientation
	void setScanner(float px, float py, float pz, float dx, float dy, float dz,
			float ux, float uy, float uz);
	//! Get scanner position and orientation
	void getScanner(float &px, float &py, float &pz, float &dx, float &dy, float &dz,
			float &ux, float &uy, float &uz);

	//! Set the data from a ROS PointCloud message.
	void setFromRosCloud(const std_msgs::PointCloud &cloud);
	//! Returns a PointCloud ros msg.
	std_msgs::PointCloud getPointCloud() const;

	//! Clears all the data inside this scan
	void clear();
	//! Write point cloud to an output stream
	void writeToFile(std::iostream &output);
	//! Write point cloud in vrml format to an output stream
	void writeToFileAsVrml(std::iostream &output);
	//! Read point cloud from an input stream. 
	bool readFromFile(std::iostream &input);
	//! Returns the number of points in the cloud
	inline int size() const {return mNumPoints;}
	//! Returns the i-th point in the cloud.
	inline std_msgs::Point32 getPoint(int i) const{
		assert( i>=0 && i<mNumPoints);
		return mNativePoints[i];
	}
	//! Returns the centroid of the point cloud
	libTF::TFPoint centroid() const;
	//! Get the principal axes of the point cloud
	void principalAxes(libTF::TFVector &a1, libTF::TFVector &a2,
			   libTF::TFVector &a3);
	//! Get the bounding box of this scan
	void boundingBox(libTF::TFPoint &bb1, libTF::TFPoint &bb2)const;

	//! Adds the points from another scan to this one
	void addScan(const SmartScan *target);
	//! Removes points from this scan that are also present in another scan
	void subtractScan(const SmartScan *target, float thresh);

	//! Apply a transform to each point in the cloud using a NEWMAT matrix
	void applyTransform(const NEWMAT::Matrix &M);
	//! Apply a transform to each point in the cloud using a row-major float[16]
	void applyTransform(float *t);

	//! Performs 2D Delaunay Triangulation on the point cloud.
	std::vector<scan_utils::Triangle> *delaunayTriangulation(double tolerance, double alpha);
	//! Performs 3D Delaunay Traingulation on the point cloud.
	std::vector<scan_utils::Triangle> *delaunayTriangulation3D(double tolerance, double alpha);
	//! Crops the point cloud to a 3D bounding box
	void crop(float x, float y, float z, float dx, float dy, float dz);
	//! Returns all the points in the scan that are within a given sphere
	std::vector<std_msgs::Point32> *getPointsWithinRadius(float x, float y, float z, float radius);
	//! Returns all the points in the scan that are within a given sphere in ros pointcloud format.
	std_msgs::PointCloud *getPointsWithinRadiusPointCloud(float x, float y, float z, float radius);
	//! Removes outliers - points that have few neighbors
	void removeOutliers(float radius, int nbrs);
	//! Removes points whose normals are perpendicular to the direction of the scanner
	void removeGrazingPoints(float threshold, bool removeOutliers = true, float radius = 0.01, int nbrs = 5);
	//! Returns the transform that registers this point cloud (computed using ICP).
	float* ICPTo(SmartScan *target);
	//! Finds the dominant plane in the point cloud by histograming point normals
	float normalHistogramPlane(std_msgs::Point32 &planePoint, std_msgs::Point32 &planeNormal,
				  float radius = 0.01, int nbrs = 5);
	//! Finds the dominant plane in the point cloud using RANSAC
	float ransacPlane(std_msgs::Point32 &planePoint, std_msgs::Point32 &planeNormal,
			 int iterations = 500, float distThresh = 0.02);
	//! Removes all points that are close to a given plane
	void removePlane(const std_msgs::Point32 &planePoint, 
			 const std_msgs::Point32 &planeNormal, float thresh = 0.02);

void removePlane(const std_msgs::Point32 &planePoint, 
			    const std_msgs::Point32 &planeNormal, float thresh, const std_msgs::PointStamped &origin, float far_remove_distance, float far_remove_distance_threshold);

	//! Computes the normal of a point by looking at its neighbors
	std_msgs::Point32 computePointNormal(int id, float radius = 0.01, int nbrs = 5);

	//! Computes the normal of a point by looking at its neighbors
	std_msgs::Point32 computePointNormal(float x, float y, float z, float radius = 0.01, int nbrs = 5);

	//! Returns the connected components in this scan, each in its own SmartScan
	std::vector<SmartScan*> *connectedComponents(float thresh, int minPts = 0);

	//! Returns a triangular surface mesh that approximates this cloud.
	std::vector<scan_utils::Triangle> *createMesh(float resolution = 0);

	//! Computes a spin image at x, y, z with fixed orientation, i.e. the surface normal is set to point up.
	bool computeSpinImageFixedOrientation(scan_utils::Grid2D &si, 
						    float x, float y, float z, float support, float pixelsPerMeter, string up);

	//! Inserts all the points in this scan in the Octree \a o with a value of \a 1.
	template <typename T>
		void insertInOctree(scan_utils::Octree<T> *o, T value);
	
	//! Computes a spin image at x, y, z using the surface normal at that point.
	bool computeSpinImageNatural(scan_utils::Grid2D &si, float x, float y, float z, float support, float pixelsPerMeter, float radius = 0.02, int nbrs = 20);
};

#endif
