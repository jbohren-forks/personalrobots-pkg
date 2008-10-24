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

#include "smartScan.h"

#include "math.h"
#include <list>
#include <algorithm>

#include "octree.h"

#include "vtk-5.0/vtkFloatArray.h"
#include "vtk-5.0/vtkPoints.h"
#include "vtk-5.0/vtkPolyData.h"
#include "vtk-5.0/vtkUnstructuredGrid.h"
#include "vtk-5.0/vtkDelaunay2D.h"
#include "vtk-5.0/vtkDelaunay3D.h"
#include "vtk-5.0/vtkCellArray.h"
#include "vtk-5.0/vtkPointLocator.h"
#include "vtk-5.0/vtkIterativeClosestPointTransform.h"
#include "vtk-5.0/vtkMatrix4x4.h"
#include "vtk-5.0/vtkCellLocator.h"
#include "vtk-5.0/vtkLandmarkTransform.h"
#include "vtk-5.0/vtkTransform.h"
#include "vtk-5.0/vtkTransformPolyDataFilter.h"
#include "vtk-5.0/vtkTransformFilter.h"
#include "vtk-5.0/vtkImplicitModeller.h"
#include "vtk-5.0/vtkPolyDataMapper.h"
#include "vtk-5.0/vtkContourFilter.h"
#include "vtk-5.0/vtkSurfaceReconstructionFilter.h"
#include "vtk-5.0/vtkMarchingCubes.h"


#include "newmat10/newmatap.h"
#include <iomanip>
#include "newmat10/newmatio.h"

#include <cmath>

using namespace scan_utils;

static const std::string SU_TFRT = "RootFrame";
static const std::string SU_TFMF = "MyFrame";

SmartScan::SmartScan()
{
	mNumPoints = 0;
	mNativePoints = NULL;
	mVtkData = NULL;
	mVtkPointLocator = NULL;
	setScanner(0,0,0,  1,0,0,  0,0,1);
}

SmartScan::~SmartScan()
{
	clearData();
}

void SmartScan::clearData()
{
	if (mNativePoints) {
		delete [] mNativePoints;
		mNativePoints = NULL;
	}
	deleteVtkData();
	mNumPoints = 0;
}

void SmartScan::clear()
{
	clearData();
	setScanner(0,0,0,  1,0,0,  0,0,1);
}

/*! Set scanner position and orientation
  \param px \param py \param pz - new scanner position
  \param dx \param dy \param dz - new scanner orientation
  \param ux \param uy \param uz - new "up" direction for scanner
  Please pass already normalised vectors for "orientation" and "up".
*/
void SmartScan::setScanner(float px, float py, float pz,
			   float dx, float dy, float dz,
			   float ux, float uy, float uz)
{
	mScannerPos.x = px; mScannerPos.y = py; mScannerPos.z = pz;
	mScannerPos.frame = SU_TFMF; mScannerPos.time = 0;
	//should normalize them too...
	mScannerDir.x = dx; mScannerDir.y = dy; mScannerDir.z = dz;
	mScannerDir.frame = SU_TFMF; mScannerDir.time = 0;
	mScannerUp.x = ux; mScannerUp.y = uy; mScannerUp.z = uz;
	mScannerUp.frame = SU_TFMF; mScannerUp.time = 0;
}

void SmartScan::getScanner(float &px, float &py, float &pz, 
			   float &dx, float &dy, float &dz,
			   float &ux, float &uy, float &uz)
{
	px = mScannerPos.x; py = mScannerPos.y; pz = mScannerPos.z;
	dx = mScannerDir.x; dy = mScannerDir.y; dz = mScannerDir.z;
	ux = mScannerUp.x; uy = mScannerUp.y; uz = mScannerUp.z;
}

void SmartScan::setPoints(int numPoints, const std_msgs::Point32 *points)
{
	// for now we are making explicit copies, but later this will be changed to not
	// copy the data but just store it
	clearData();
	mNumPoints = numPoints;
	assert(mNumPoints >= 0);
	mNativePoints = new std_msgs::Point32[mNumPoints];
	for (int i=0; i<mNumPoints; i++) {
		mNativePoints[i] = points[i];
	}
}

/*!  Set the points using an array of floats.  
  \param numPoints - number of 3D vertices 
  \param points - float array of size 3 * \a numPoints, holding each vertex as x,y,z
*/
void SmartScan::setPoints(int numPoints, const float *points)
{
	clearData();
	mNumPoints = numPoints;
	assert(mNumPoints >= 0);
	mNativePoints = new std_msgs::Point32[mNumPoints];
	for (int i=0; i<mNumPoints; i++) {
		mNativePoints[i].x = points[3*i+0];
		mNativePoints[i].y = points[3*i+1];
		mNativePoints[i].z = points[3*i+2];
	}
}

/*! Set the data from a ROS PointCloud message. For the moment,
    it ingores all channels except the points themselves.
*/
void SmartScan::setFromRosCloud(const std_msgs::PointCloud &cloud)
{
	//for the moment we ignore intensity values
	setPoints( cloud.get_pts_size(), &cloud.pts[0]);
}

/*!  Returns the SmartScan as a ROS PointCloud message.
  Currently, this sets the intensity channel to all zeros.  
*/
std_msgs::PointCloud SmartScan::getPointCloud() const{
	std_msgs::PointCloud cloud;
	cloud.set_pts_size(mNumPoints);
	cloud.set_chan_size(1);
	cloud.chan[0].name = "intensities";
	cloud.chan[0].set_vals_size(mNumPoints);
	for(int i=0; i<mNumPoints; i++) {
		std_msgs::Point32 pt = getPoint(i);
		cloud.pts[i].x = pt.x;
		cloud.pts[i].y = pt.y;
		cloud.pts[i].z = pt.z;
		cloud.chan[0].vals[i] = 0;
	}
	return cloud;
}

std_msgs::Point32 SmartScan::transformPoint(const std_msgs::Point32 &p_in,
						   libTF::TransformReference &tr) const
{

	libTF::TFPoint p;
	p.x = p_in.x; p.y = p_in.y; p.z = p_in.z;
	p.frame = SU_TFMF;	p.time = 0;
	p = tr.transformPoint(SU_TFRT, p);

	std_msgs::Point32 p_out;
	p_out.x = p.x; p_out.y = p.y; p_out.z = p.z;
	//fprintf(stderr,"Transformed: %f %f %f\n",p.x, p.y, p.z);
	return p_out;
}


void SmartScan::applyTransform(const NEWMAT::Matrix &M)
{
	//we could take advantage of the fact that libTF can do these things for us
	//but for now I'll just multiply the matrices myself because I'm still a bit
	//unsure about how to use libTF

	libTF::TransformReference tr;
	tr.setWithMatrix( SU_TFMF, SU_TFRT, M, 0);

	for (int i=0; i<mNumPoints; i++) {
		mNativePoints[i] = transformPoint( mNativePoints[i], tr );
	}

	mScannerPos = tr.transformPoint(SU_TFRT, mScannerPos);
	mScannerDir = tr.transformVector(SU_TFRT, mScannerDir);
	mScannerUp = tr.transformVector(SU_TFRT, mScannerUp);
	mScannerPos.frame = SU_TFMF; mScannerPos.time = 0;
	mScannerDir.frame = SU_TFMF; mScannerDir.time = 0;
	mScannerUp.frame = SU_TFMF; mScannerUp.time = 0;

	if (hasVtkData()) {
		deleteVtkData();
		createVtkData();
	}
}

void SmartScan::applyTransform(float *t)
{
	NEWMAT::Matrix M(4,4);
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++) {
			M.element(i,j) = t[i*4+j];
		}
	}
	applyTransform(M);
}

/*!
  Writes the data in this scan to an output stream. Format is:
  
  number of points
  one point per line as  x y z intensity
*/
void SmartScan::writeToFile(std::iostream &output)
{
	//write points
	float intensity = 0;
	output << mNumPoints << std::endl;
	for (int i=0; i < mNumPoints; i++) {
		output << mNativePoints[i].x << " " << mNativePoints[i].y << " " 
		       << mNativePoints[i].z << " " << intensity << std::endl;
	}
	fprintf(stderr,"Written %d points to file\n", mNumPoints);
}

/*!
  Writes the data in this scan to an output stream in VRML format
 */
void SmartScan::writeToFileAsVrml(std::iostream &output)
{
	std_msgs::Point32 p;
	output << "#VRML V2.0 utf8" << std::endl;
	output << "Shape" << std::endl << "{" << std::endl;
	output << "geometry PointSet" << std::endl << "{" << std::endl;
	output << "coord Coordinate" << std::endl << "{" << std::endl;
	output << "point[" << std::endl;
	for (int i=0; i < mNumPoints; i++) {
		p = getPoint(i);
		output << p.x << " " << p.y << " " << p.z << std::endl;
	}
	output << "]" << std::endl; //point[
	output << "}" << std::endl; //Coordinate{
	output << "}" << std::endl; //PointSet{
	output << "}" << std::endl; //Shape{
}

/*!  
  Reads in data from an input stream. Format is expected to be
  identical to the one created by \a writeToFile(...)
*/
bool SmartScan::readFromFile(std::iostream &input)
{
	int n;
	float x,y,z,intensity;
	clearData();
	//we alse reset scanner position here
	setScanner(0,0,0,  1,0,0,  0,0,1);
	//read number of points
	input >> n;
	if (n <= 0) {
		return false;
	}
	//read point data
	mNumPoints = n;
	mNativePoints =  new std_msgs::Point32[mNumPoints];

	int i;
	for (i=0; i<mNumPoints; i++) {
		if (input.eof()) break;
		input >> x >> y >> z >> intensity;
		mNativePoints[i].x = x;
		mNativePoints[i].y = y;
		mNativePoints[i].z = z;

	}
	if (i!=mNumPoints) return false;
	return true;
}

void SmartScan::createVtkData()
{
	if (hasVtkData()) deleteVtkData();

	// Create a float array which represents the points.
	vtkFloatArray* pcoords = vtkFloatArray::New();
	// Note that by default, an array has 1 component.
	// We have to change it to 3 for points
	pcoords->SetNumberOfComponents(3);
	// We ask pcoords to allocate room for all the tuples we need
	pcoords->SetNumberOfTuples(mNumPoints);
	// Assign each tuple. 
	for (int i=0; i<mNumPoints; i++){
		pcoords->SetTuple3(i, mNativePoints[i].x, mNativePoints[i].y, mNativePoints[i].z);
	}
	// Create vtkPoints and assign pcoords as the internal data array.
	vtkPoints* points = vtkPoints::New();
	points->SetData(pcoords);
	// Create vtkPointSet and assign vtkPoints as internal data
	mVtkData = vtkPolyData::New();
	mVtkData->SetPoints(points);

	//for some functions it seems we also need the points represented as "cells"...
	vtkCellArray *cells = vtkCellArray::New();
	for (int i=0; i<mNumPoints; i++) {
		cells->InsertNextCell(1);
		cells->InsertCellPoint(i);
	}
	mVtkData->SetVerts(cells);

	//also create and populate point locator
	mVtkPointLocator = vtkPointLocator::New();
	mVtkPointLocator->SetDataSet(mVtkData);
	mVtkPointLocator->BuildLocator();

	//fprintf(stderr,"Automatic: %d\n",mVtkPointLocator->GetAutomatic());
	//fprintf(stderr,"p per b: %d\n",mVtkPointLocator->GetNumberOfPointsPerBucket());
}

void SmartScan::deleteVtkData()
{
	if (!mVtkData) return;
	mVtkData->Delete();
	mVtkData = NULL;
	assert(mVtkPointLocator);
	mVtkPointLocator->Delete();
	mVtkPointLocator = NULL;
}

vtkPolyData* SmartScan::getVtkData()
{
	if (!hasVtkData()) createVtkData();
	assert(mVtkData);
	return mVtkData;
}

vtkPointLocator* SmartScan::getVtkLocator()
{
	if (!hasVtkData()) createVtkData();
	assert(mVtkPointLocator);
	return mVtkPointLocator;
}

libTF::TFPoint SmartScan::centroid() const
{
	std_msgs::Point32 p;
	libTF::TFPoint c;
	c.time = 0; c.frame = SU_TFMF;
	c.x = c.y = c.z = 0;

	if (size()==0) return c;
	for (int i=0; i<size(); i++) {
		p = getPoint(i);
		c.x += p.x; c.y += p.y; c.z += p.z;
	}
	c.x = c.x / size();
	c.y = c.y / size();
	c.z = c.z / size();
	return c;
}

/*!  Returns the principal axes of the point cloud in \a a1, \a a2 and
  \a a3, ordered from most significant to least significant.

  Principal axes are computed by doing SVD on the normalized matrix of
  components.

  WARNING: singular vectors are orthonormal, but there's no guarantee
  they form a right-handed coordinate system. This function might flip
  \a a3 in order to ensure a right-handed coord. system.

  Future work: make this more robust by disregarding outliers.
 */
void SmartScan::principalAxes(libTF::TFVector &a1, libTF::TFVector &a2,
			      libTF::TFVector &a3)
{
	std_msgs::Point32 p;
	libTF::TFPoint c = centroid();

	//asemble normalized NEWMAT matrix
	NEWMAT::Matrix M(size(),3);
	for (int i=0; i<size(); i++) {
		p = getPoint(i);
		M.element(i,0) = p.x - c.x;
		M.element(i,1) = p.y - c.y;
		M.element(i,2) = p.z - c.z;
	}

	//for now we use a mixture of Point32 and libTF::TFVector
	//this is just a hack, I need to clean this up in the future
	std_msgs::Point32 pa1, pa2, pa3;

	//do SVD decomposition
	singularVectors(&M,size(),pa1,pa2,pa3);

	//  singular vectors are orthonormal, but there's no guarantee
	//  they form a right-handed coordinate system! We might need to flip one of the vectors.
	if ( dot ( cross(pa1,pa2) , pa3 ) < 0) {
		pa3.x = -pa3.x; pa3.y = -pa3.y; pa3.z = -pa3.z;
	}

	//and copy back into the libTF::TFVectors. Horrible hack.
	a1.x = pa1.x; a1.y = pa1.y; a1.z = pa1.z;
	a2.x = pa2.x; a2.y = pa2.y; a2.z = pa2.z;
	a3.x = pa3.x; a3.y = pa3.y; a3.z = pa3.z;

	//set frame and time to 0. In the future, a smart scan might have its own frame and time
	a1.time = a2.time = a3.time = 0;
	a1.frame = a2.frame = a3.frame = SU_TFMF;
}

/*! Return the two corners of the bounding box of this scan, in \a bb1
    and \a bb2 respectively. Computes the bounding box each time it is
    called; in the future we could also compute the bbox when the
    points are set and then cache it.
 */
void SmartScan::boundingBox(libTF::TFPoint &bb1, libTF::TFPoint &bb2) const
{
	if (size() < 0) return;
	std_msgs::Point32 p = getPoint(0);
	bb1.x = bb2.x = p.x;
	bb1.y = bb2.y = p.y;
	bb1.z = bb2.z = p.z;

	for (int i=1; i<size(); i++) {
		p = getPoint(i);

		bb1.x = std::min((float)bb1.x, p.x);
		bb1.y = std::min((float)bb1.y, p.y);
		bb1.z = std::min((float)bb1.z, p.z);

		bb2.x = std::max((float)bb2.x, p.x);
		bb2.y = std::max((float)bb2.y, p.y);
		bb2.z = std::max((float)bb2.z, p.z);
	}
}

/*! Adds the points in the scan \a target to this one. Does not check
    for duplicate points.
 */
void SmartScan::addScan(const SmartScan *target)
{
	if ( !target->size() ) return;

	std_msgs::Point32 *newPoints = new std_msgs::Point32[mNumPoints + target->size()];

	int i;
	for (i=0; i<mNumPoints; i++) {
		newPoints[i] = getPoint(i);
	}
	for (i=0; i<target->size(); i++) {
		newPoints[mNumPoints+i] = target->getPoint(i);
	}
	setPoints(mNumPoints + target->size(), newPoints);
}


/*! Performs a 2D Delaunay triangulation of the point cloud. This is
    equivalent to projecting all point onto the x-y plane (by dropping
    the z coordinate), performing the 2D triangulation then pushing
    the resulting triangles back into 3D space by adding the z
    coordinate back.  

    \param tolerance Points that are closer than this value (after
    projection on x-y plane) will be merged together

    \param alpha No points that are further apart than this value
    will be joined by a triangle

    The output lists the triangles produced by triangulation.  It is
    the responsability of the caller to free this memory.
 */
std::vector<Triangle> *SmartScan::delaunayTriangulation(double tolerance, double alpha)
{
	std::vector<Triangle> *triangles = new std::vector<Triangle>;

	// create the Delaunay triangulation filter
	vtkDelaunay2D *delny = vtkDelaunay2D::New();
	// set our point set as input
	delny->SetInput( getVtkData() );
	delny->SetTolerance(tolerance);
	delny->SetAlpha(alpha);

	// get the output
	vtkPolyData *output = delny->GetOutput();
	// run the filter
	delny->Update();

	//we should now have the result in the output
	int nTri = output->GetNumberOfStrips();
	int nPol = output->GetNumberOfPolys();
	fprintf(stderr,"Delaunay: %d tris and %d polys\n",nTri,nPol);

	//it seems output is as polygons
	vtkCellArray *polygons = output->GetPolys();
	polygons->InitTraversal();
	int nPts, *pts;
	double coords[9];
	while( polygons->GetNextCell(nPts,pts) ){
		if (nPts!=3) {
			fprintf(stderr,"Delaunay cell does not have 3 points\n");
			continue;
		}
		output->GetPoint( pts[0], &(coords[0]) );
		output->GetPoint( pts[1], &(coords[3]) );
		output->GetPoint( pts[2], &(coords[6]) );
		triangles->push_back( Triangle(coords) );

	}

	delny->Delete();
	return triangles;
}

/*! Performs a 3D Delaunay triangulation of the point cloud. This
    actually means creating a volume composed of tetrahedra that best
    approximates the cloud in a Delaunay sense.

    \param tolerance Points that are closer than this value (after
    projection on x-y plane) will be merged together

    \param alpha No points that are further apart than this values
    will be joined by a tetrahedron.

    The output list all four triangles that compose the each of the
    resulting tetrahedra. It automatically contains a surface mesh,
    but many of the returned triangles will be "inside" the volume
    produced.  It is the responsability of the caller to free this
    memory.
 */
std::vector<Triangle> *SmartScan::delaunayTriangulation3D(double tolerance, double alpha)
{
	std::vector<Triangle> *triangles = new std::vector<Triangle>;

	// create the Delaunay triangulation filter
	vtkDelaunay3D *delny = vtkDelaunay3D::New();
	// set our point set as input
	delny->SetInput( getVtkData() );
	// tolerance is distance that nearly coincident points are merged together
	delny->SetTolerance(tolerance);
	// I'm not really sure what this Alpha thing is
	delny->SetAlpha(alpha);

	// get the output
	vtkUnstructuredGrid *output = delny->GetOutput();
	// run the filter
	delny->Update();

	vtkCellArray *cells = output->GetCells();
	cells->InitTraversal();
	int nPts, *pts, tets=0;
	double c1[3], c2[3], c3[3], c4[3];

	while( cells->GetNextCell(nPts,pts) ){
		if (nPts!=4) {
			//not a tetrahedron
			continue;
		}
		output->GetPoint( pts[0], c1 );
		output->GetPoint( pts[1], c2 );
		output->GetPoint( pts[2], c3 );
		output->GetPoint( pts[3], c4 );
		triangles->push_back( Triangle(c2, c1, c3) );
		triangles->push_back( Triangle(c1, c2, c4) );
		triangles->push_back( Triangle(c3, c1, c4) );
		triangles->push_back( Triangle(c2, c3, c4) );
		tets++;
	}
	fprintf(stderr,"3D Delaunay: %d tets\n",tets);
	delny->Delete();
	return triangles;
}

/*! Crops the point cloud to a 3D bounding box

  \param x, y, z The location of the center of the crop
  bounding box

  \param dx, dy, dz The dimensions of the crop bounding
  box.
*/
void SmartScan::crop(float x, float y, float z, float dx, float dy, float dz)
{
	std_msgs::Point32 *newPoints,p;
	int numNewPoints = 0;
	
	//we are passed the overall size of the crop box; let's get the halves
	dx = dx/2; dy = dy/2; dz = dz/2;

	//we are doing two passes; one to count how many points we are keeping
	//(so we can allocate memory) and one to actually copy the points

	for (int i=0; i<mNumPoints; i++) {
		//we are getting the point already transformed by the inner transform
		//here we use VTK to transform the point instead of our own this->getPoint(...)
		p = getPoint(i);
		if (p.x > x + dx) continue;
		if (p.x < x - dx) continue;
		if (p.y > y + dy) continue;
		if (p.y < y - dy) continue;
		if (p.z > z + dz) continue;
		if (p.z < z - dz) continue;
		numNewPoints++;
	}
	newPoints = new std_msgs::Point32[numNewPoints];
	int j=0;
	for (int i=0; i<mNumPoints; i++) {
		p = getPoint(i);
		if (p.x > x + dx) continue;
		if (p.x < x - dx) continue;
		if (p.y > y + dy) continue;
		if (p.y < y - dy) continue;
		if (p.z > z + dz) continue;
		if (p.z < z - dz) continue;
		newPoints[j] = getPoint(i);
		j++;
	}
	assert(j==numNewPoints);
	fprintf(stderr,"Cropped from %d to %d points\n",mNumPoints, numNewPoints);

	setPoints(numNewPoints, newPoints);

	if ( hasVtkData() ) {
		deleteVtkData();
		createVtkData();
	}

}

/*! Computes the transform that registers this scan to another scan.
  
  \param target Scan that we are registering against

  Returns the transform as a 4x4 matrix saved in a float[16] in
  row-major order. It is the responsability of the caller to free this
  memory.

  Uses a VTK implementation which is very simple. For example it
  requires well cleaned scans so that all objects present in one scan
  are also present in the other.
 */
float* SmartScan::ICPTo(SmartScan* target)
{
	vtkIterativeClosestPointTransform *vtkTransform = vtkIterativeClosestPointTransform::New();
	vtkTransform->SetMaximumNumberOfIterations(1000);
	vtkTransform->SetMaximumNumberOfLandmarks(500);
	fprintf(stderr,"max iterations: %d\n",vtkTransform->GetMaximumNumberOfIterations());

	vtkTransform->SetSource( getVtkData() );
	vtkTransform->SetTarget( target->getVtkData() );
	vtkTransform->GetLandmarkTransform()->SetModeToRigidBody();

	fprintf(stderr,"Iterations used at beginning: %d\n", vtkTransform->GetNumberOfIterations() );
	fprintf(stderr,"Points: %d and %d\n",this->getVtkData()->GetNumberOfPoints(), 
		target->getVtkData()->GetNumberOfPoints() );


	float* transf = new float[16];
	vtkMatrix4x4 *vtkMat = vtkMatrix4x4::New();
	vtkTransform->GetMatrix(vtkMat);

	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++) {
			transf[4*i+j] = vtkMat->GetElement(i,j);
		}
	}

	fprintf(stderr,"ICP done. Result:\n");
	vtkMat->Print(std::cerr);
	fprintf(stderr,"Iterations used: %d. Mean dist: %f\n",vtkTransform->GetNumberOfIterations(),
		vtkTransform->GetMeanDistance());

	vtkTransform->Delete();
	vtkMat->Delete();
	return transf;
}

/*! Removes all points that have fewer than \a nbrs neighbors
    within a sphere of radius \a radius.
 */
void SmartScan::removeOutliers(float radius, int nbrs)
{
	if ( size() == 0) return;
	std_msgs::Point32 p;
	if (! hasVtkData() ) createVtkData();
	vtkIdList *result = vtkIdList::New();

	// allocate memory as if we will keep all points, but later we'll do a copy and a delete
	std_msgs::Point32 *newPoints = new std_msgs::Point32[mNumPoints];
	int numNewPoints = 0;
	for (int i=0; i<mNumPoints; i++) {
		result->Reset();
		p = getPoint(i);
		getVtkLocator()->FindPointsWithinRadius( radius,p.x, p.y, p.z, result );
		//there is always at least one point in the result (the point itself)
		if (result->GetNumberOfIds() > nbrs) {
			//keep this point
			newPoints[numNewPoints] = getPoint(i);
			numNewPoints++;
		}
	}
	result->Delete();

	//keep only the new points
	fprintf(stderr,"Removed outliers from %d to %d\n",mNumPoints,numNewPoints);
	//this will copy the points again, so we are doing two copies in order to only
	//compute the nbrs once
	setPoints(numNewPoints,newPoints);
	//we can now delete this array which has worng size anyway
	delete [] newPoints;
	//and re-create vtk data (which does yet another copy...)
	createVtkData();
}

/*!  Removes all points whose normals are perpendicular to the scanner
  direction. Scanner position is set using \a setScanner(...);
  default is \a (0,0,0) which is the case for native Hokuyo scans. The
  assumption is that points with this characteristic are scanned with
  high error or are "ghosting" or "veil" artifacts.

  \param threshold The minimum difference in degrees between normal
  direction and scanner perpendicular direction for keeping
  points. For example, if \a threshold = 10 all points whose
  normal is closer than 10 degrees to beeing perpendicular to the
  scanner direction are removed.

  \param removeOutliers What do we do with points that have too few
  neighbors to compute normals: if this flag is true, these points are
  removed (as in \a removeOutliers(...) )

  For computing point normals we look for at least \a nbrs neighbors
  within a sphere of radius \a radius.
 */
void SmartScan::removeGrazingPoints(float threshold, bool removeOutliers, float radius, int nbrs)
{
	if ( size() == 0) return;
	// allocate memory as if we will keep all points, but later we'll do a copy and a delete
	std_msgs::Point32 *newPoints = new std_msgs::Point32[mNumPoints];
	int numNewPoints = 0;

	//we get the threshold in degrees
	threshold = fabs( threshold * M_PI / 180.0 );
	threshold = fabs( cos(M_PI / 2 - threshold) );

	std_msgs::Point32 scanner;
	scanner.x = mScannerPos.x; 
	scanner.y = mScannerPos.y; 
	scanner.z = mScannerPos.z;

	for (int i=0; i<mNumPoints; i++) {
		std_msgs::Point32 normal = computePointNormal(i,radius,nbrs);
		//check for outliers
		if ( norm(normal) < 0.5 && removeOutliers) continue;
	
		//compute direction to scanner
		std_msgs::Point32 scannerDirection;
		std_msgs::Point32 p = getPoint(i);
		scannerDirection.x = p.x - scanner.x;
		scannerDirection.y = p.y - scanner.y;
		scannerDirection.z = p.z - scanner.z;
		scannerDirection = normalize(scannerDirection);

		//we don't care about direction; will check dot product in absolute value
		float d = fabs( dot(normal, scannerDirection) );
	
		if ( norm(normal) > 0.5 && d < threshold) continue; 
		//keep the point
		newPoints[numNewPoints] = getPoint(i);
		numNewPoints++;
	}

	//keep only the new points
	fprintf(stderr,"Removed grazing points from %d to %d\n",mNumPoints,numNewPoints);
	//this will copy the points again, so we are doing two copies in order to only
	//compute the nbrs once
	setPoints(numNewPoints,newPoints);
	//we can now delete this array which has worng size anyway
	delete [] newPoints;
	//and re-create vtk data (which does yet another copy...)
	createVtkData();
}


/*!  Computes the dominant plane in the scan by histograming point
  normals. First, all point normals are computed and
  histogramed. Then, the dominant normal direction is selected. Then,
  for all points that share this normal, the plane distance from the
  origin is histogramed.

  This approach is much less memory intensive than a traditional Hough
  transform as it performs a 2D histogram and a 1D histogram rather
  than a 3D histogram. However, it is also less accurate.

  \param planePoint, planeNormal Output values, holding the plane found by the function

  For computing point normals we look for at least \a nbrs neighbors
  within a sphere of radius \a radius.

  Returns a measure of how many points in the cloud are close to the
  returned plane. This is the exact value of the histogram at its
  highest points, but as such it does not have clear units. Will
  generally be somewhere between 0 and 2, with higher values
  indicating that more points in the cloud lie close to the plane.
*/
float SmartScan::normalHistogramPlane(std_msgs::Point32 &planePoint, std_msgs::Point32 &planeNormal,
				     float radius, int nbrs)
{
	std_msgs::Point32 zero; zero.x = zero.y = zero.z = 0.0;
	if ( size() == 0) {
		planeNormal = zero; planePoint = zero; return 0;
	}

	float binSize = 1.0 * M_PI / 180.0;

	float maxDist = 10.0;
	float distBinSize = 0.01;

	int d1 = ceil( (M_PI/2.0) / binSize);
	int d2 = ceil( 2.0 * M_PI / binSize);
	int dd = ceil( 2.0 * maxDist / distBinSize );

	int max,b1,b2,sb1,sb2,sbd; float theta, phi, dist;

	//prepare grid for normals
	Grid2D *grid2 = new Grid2D(d1,d2);
	//prepare grid for distances
	Grid1D *grid1 = new Grid1D(dd);

	//histogram normals
	for (int i=0; i<mNumPoints; i++) {
		std_msgs::Point32 normal = computePointNormal(i,radius,nbrs);
		if ( norm(normal) < 0.5 ) continue;
		//we only care about a halfspace. This should also ensure theta <= M_PI/2
		if(normal.z < 0) {
			normal.x = -normal.x;
			normal.y = -normal.y;
			normal.z = -normal.z;
		}
		theta = acos(normal.z);
		phi = atan2( normal.y, normal.x );
		//fprintf(stderr,"theta %f and phi %f\n",theta, phi);
		if (phi < 0) phi += 2 * M_PI;

		b1 = floor(theta / binSize);
		b2 = floor(phi / binSize);

		//try to get around singularity problem with spherical coords
		if ( b1 == 0) b2 = 0;

		grid2->addGrid(b1,b2,&Grid2D::MASK);
	}

	//take largest value
	grid2->getMax(max, sb1, sb2);
	theta = sb1 * binSize;
	phi = sb2 * binSize;

	planeNormal.x = sin(theta) * cos(phi);
	planeNormal.y = sin(theta) * sin(phi);
	planeNormal.z = cos(theta);

	fprintf(stderr,"Normal theta %f and phi %f with %d votes\n",theta, phi, max);

	//histogram distances
	for(int i=0; i<mNumPoints; i++) {
		std_msgs::Point32 normal = computePointNormal(i,radius,nbrs);
		if ( norm(normal) < 0.5 ) continue;
		double d = fabs( dot(normal, planeNormal) );
		if ( d < 0.986 ) continue; //10 degrees as threshold 
		//histogram the point
		if(normal.z < 0) {
			normal.x = -normal.x;
			normal.y = -normal.y;
			normal.z = -normal.z;
		}
		dist = dot(planeNormal, getPoint(i) );
		if (dist < -maxDist) dist = -maxDist;
		if (dist > maxDist) dist = maxDist;
		b1 = floor( (dist + maxDist)/distBinSize );
		grid1->addGrid(b1,&Grid1D::MASK);
	}

	//find plane
	grid1->getMax(max,sbd);
	dist = sbd * distBinSize - maxDist;

	planePoint.x = dist * planeNormal.x;
	planePoint.y = dist * planeNormal.y;
	planePoint.z = dist * planeNormal.z;

	fprintf(stderr,"Distance %f with %d votes\n",dist,max);

	delete grid2;
	delete grid1;

	return ((float)max) / size();
}

/*! Computes the dominant plane in the scan using RANSAC.

  \param planePoint, planeNormal Output values, holding the plane found by the function

  \param iterations Number of RANSAC iterations to be performed

  \param distThresh Internal RANSAC threshold: points that are closer
  than this value to a hypothesis plane are considered inliers

  Returns the ratio of points that are within \a distThresh of the
  plane to total number of points in the scan. This can be considered
  as a measure of how dominant the plane is in the scan.

 */
float SmartScan::ransacPlane(std_msgs::Point32 &planePoint, std_msgs::Point32 &planeNormal,
			    int iterations, float distThresh)
{
	std_msgs::Point32 foo,zero; zero.x = zero.y = zero.z = 0.0;

	if ( size() == 0) {
		planeNormal = zero; planePoint = zero; return 0;
	}
	int selPoints = 3;

	//if this is true, each plane is re-fit to all inliers after inliers are calulated
	//otehrwise, planes are just fit to the initial random chosen points.
	bool refitToConsensus = true;

	int it = 0, consensus, maxConsensus = 0;
	int i,k;
	float dist;

	NEWMAT::Matrix M(selPoints,3);
	std_msgs::Point32 mean, consMean, normal, dif, p;
	std::list<std_msgs::Point32> consList;

	//seed random generator
	srand( (unsigned)time(NULL) );

	while (1) {
		//randomly select selPoints from data
		//compute the centroid as well while we're at it
		mean = zero;
		for (i=0; i<selPoints; i++) {
			int index = floor( ((double)(mNumPoints-1)) * ( (double)rand() / RAND_MAX ) );
			assert (index >=0 && index < mNumPoints);
			p = getPoint(index);

			mean.x += p.x; mean.y += p.y; mean.z += p.z;
			M.element(i,0) = p.x; M.element(i,1) = p.y; M.element(i,2) = p.z;
		}
		mean.x = mean.x / selPoints; mean.y = mean.y / selPoints; mean.z = mean.z / selPoints;
		for (i=0; i<selPoints; i++) {
			M.element(i,0) = M.element(i,0) - mean.x;
			M.element(i,1) = M.element(i,1) - mean.y;
			M.element(i,2) = M.element(i,2) - mean.z;
		}

		//fit a plane to the points
		singularVectors(&M, selPoints, foo, foo, normal);

		//find all other consensus points
		consensus = 0;
		if (refitToConsensus) {
			consList.clear();
			consMean = zero;
		}
		for(int i=0; i<mNumPoints; i++) {
			p = getPoint(i);
			dif.x = p.x - mean.x;
			dif.y = p.y - mean.y;
			dif.z = p.z - mean.z;
			dist = dot(dif,normal);
			if ( fabs(dist) > distThresh ) continue;

			consensus++;
			if (refitToConsensus) {
				consList.push_back(p);
				consMean.x += p.x;
				consMean.y += p.y;
				consMean.z += p.z;
			}
		}
		if (refitToConsensus) {
			consMean.x /= consensus;
			consMean.y /= consensus;
			consMean.z /= consensus;
		}

		//break for low number of consenting points. Has to be at least 3 if 
		//refitToConsensus is being used (otehrwise can not to SVD)
		if (consensus < 3) continue;

		//save solution if it is the best
		if (consensus >= maxConsensus) {
			assert(consensus >= 3);
			maxConsensus = consensus;
			if (refitToConsensus) {
				NEWMAT::Matrix *CM = new NEWMAT::Matrix(consensus, 3);
				std::list<std_msgs::Point32>::iterator it;
				for (k=0, it = consList.begin(); it!=consList.end(); it++, k++) {
					CM->element(k,0) = (*it).x - consMean.x;
					CM->element(k,1) = (*it).y - consMean.y;
					CM->element(k,2) = (*it).z - consMean.z;
				}
				singularVectors(CM,consensus, foo, foo, planeNormal);
				planePoint = consMean;
				delete CM;
			} else {
				planePoint = mean;
				planeNormal = normal;
			}
		}

		it++;
		if (it >= iterations) break;
	}
	consList.clear();
	fprintf(stderr,"RANSAC consensus: %d\n",maxConsensus);
	return ((float)maxConsensus) / size();
}

/*!  Removes all points that are closer than \a thresh to the plane
  defined by \a planePoint and \a planeNormal
 */
void SmartScan::removePlane(const std_msgs::Point32 &planePoint, 
			    const std_msgs::Point32 &planeNormal, float thresh)
{
	//remove points that are close to plane
	std_msgs::Point32 *newPoints = new std_msgs::Point32[mNumPoints];
	std_msgs::Point32 dif, p;
	int numNewPoints = 0;
	float dist;
	for(int i=0; i<mNumPoints; i++) {
		p = getPoint(i);
		dif.x = p.x - planePoint.x;
		dif.y = p.y - planePoint.y;
		dif.z = p.z - planePoint.z;
		dist = dot(dif,planeNormal);
		if ( fabs(dist) < thresh ) continue;

		//keep this point
		newPoints[numNewPoints] = getPoint(i);
		numNewPoints++;
	}

	fprintf(stderr,"Kept %d out of %d points\n",numNewPoints,mNumPoints);
	setPoints(numNewPoints,newPoints);
	delete [] newPoints;
}

/*!  Returns all the points in this cloud that are within \a radius of
  the 3D point at( \a x \a y \a z ). Result is returned as a \a
  std::vector<Point32>* - it is the responsability of the
  caller to free this memory.
*/
std::vector<std_msgs::Point32>* SmartScan::getPointsWithinRadius(float x, float y, float z, float radius)
{
	std::vector<std_msgs::Point32> *resPts = new std::vector<std_msgs::Point32>;
	vtkIdList *result = vtkIdList::New();

	getVtkLocator()->FindPointsWithinRadius( radius, x, y, z, result );
	int n = result->GetNumberOfIds();
	resPts->reserve(n);
	for (int i=0; i<n; i++){
		int nbrId = result->GetId(i);
		resPts->push_back(getPoint(nbrId));
	}

	result->Delete();
	return resPts;
}

std_msgs::PointCloud* SmartScan::getPointsWithinRadiusPointCloud(float x, float y, float z, float radius)
{

	std_msgs::PointCloud *resPts = new std_msgs::PointCloud;
	vtkIdList *result = vtkIdList::New();

	getVtkLocator()->FindPointsWithinRadius( radius, x, y, z, result );
	int n = result->GetNumberOfIds();

	resPts->set_pts_size(n);
	resPts->set_chan_size(1);
	resPts->chan[0].name = "intensity";
	resPts->chan[0].set_vals_size(n);
	for (int i=0; i<n; i++){
		int nbrId = result->GetId(i);
		std_msgs::Point32 p = getPoint(nbrId);
		resPts->pts[i].x = p.x;
		resPts->pts[i].y = p.y;
		resPts->pts[i].z = p.z;
		resPts->chan[0].vals[i] = 0; //Placeholder for intensity.
	}

	result->Delete();
	return resPts;
}

/*!Computes the normal of the point with index \a id by fitting a
   plane to neighboring points. It uses all neighbors within a sphere
   of radius \a radius. If less than \a nbrs such neighbors
   are found, the normal is considered unreliable and the function
   returns (0,0,0)

 */
std_msgs::Point32 SmartScan::computePointNormal(int id, float radius, int nbrs)
{
        assert(id >=0 && id < mNumPoints);
	std_msgs::Point32 p = getPoint(id);
	return computePointNormal(p.x, p.y, p.z, radius, nbrs);
}


/*!Computes the normal of the point at \a x, \a y, \a z by fitting a
   plane to neighboring points. It uses all neighbors within a sphere
   of radius \a radius. If less than \a nbrs such neighbors
   are found, the normal is considered unreliable and the function
   returns (0,0,0)

 */
std_msgs::Point32 SmartScan::computePointNormal(float x, float y, float z, float radius, int nbrs)
{
	// radius - how large is the radius in which we look for nbrs for computing normal
	// nbrs - min number of nbrs we use for normal computation
	int nbrId;
	vtkIdList *result = vtkIdList::New();
	std_msgs::Point32 zero; zero.x = zero.y = zero.z = 0.0;

	getVtkLocator()->FindPointsWithinRadius( radius, x, y, z, result );
	//we don't have enough nbrs for a reliable normal
	int n = result->GetNumberOfIds();
	if ( n < nbrs ) {
		result->Delete();
		return zero;
	}

	//find the mean point for normalization
	std_msgs::Point32 mean = zero;
	for (int i=0; i<n; i++) {
		nbrId = result->GetId(i);
		assert(nbrId >= 0 && nbrId < mNumPoints);
		std_msgs::Point32 p = getPoint(nbrId);

		mean.x += p.x;
		mean.y += p.y;
		mean.z += p.z;
	}
	mean.x = mean.x / n; mean.y = mean.y / n; mean.z = mean.z / n;
	//fprintf(stderr,"%f %f %f \n",mean.x, mean.y, mean.z);

	//create and populate matrix with normalized nbrs
	NEWMAT::Matrix M(n,3);
	for (int i=0; i<n; i++) {
		nbrId = result->GetId(i);
		assert(nbrId >= 0 && nbrId < mNumPoints);

		std_msgs::Point32 p = getPoint(nbrId);

		M.element(i,0) = p.x - mean.x;
		M.element(i,1) = p.y - mean.y;
		M.element(i,2) = p.z - mean.z;
	}
	result->Delete();

	std_msgs::Point32 foo, normal;
	singularVectors(&M, n, foo, foo, normal);
	return normal;
}


/*! Given an \a n by 3 matrix \a M in NEWMAT format, performs
  SVD and returns the singular vectors in decreasing order of the singular values.

  If the matrix is a list of vertices, the last singular vector
  corresponds to the direction of the normal of a plane fit to those
  points. Vertices should be normalized first (of mean 0).
*/
void SmartScan::singularVectors(NEWMAT::Matrix *M, int n, std_msgs::Point32 &sv1, 
				std_msgs::Point32 &sv2, std_msgs::Point32 &sv3)

{
	NEWMAT::Matrix U(n,3);
	NEWMAT::DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	SVD(*M, D, U, V);

	sv1.x = V.element(0,0); sv1.y = V.element(1,0); sv1.z = V.element(2,0);
	sv2.x = V.element(0,1); sv2.y = V.element(1,1); sv2.z = V.element(2,1);
	sv3.x = V.element(0,2); sv3.y = V.element(1,2); sv3.z = V.element(2,2);
	//std::cout << *M << std::endl;
	//std::cout << U << std::endl;
	//std::cout << D << std::endl;
	//std::cout << V << std::endl;
	//fprintf(stderr,"%f %f %f \n",normal.x, normal.y, normal.z);
}

/*! Returns a triangular mesh that approximates this point cloud.The
    mesh will usually be closed and wrapping around the point cloud,
    thus creating acomplete 3D object, not just one face of it. This
    is generally a drawback, but for now we don't have a better
    meching algorithm available.

    The scan is sampled into a regular grid, which is then in turn meshed.
    
    \param resolution - the size of a grid cell. Usually values around
    0.01 work well for regular sized objects. If a 0 is passed (which
    is also the default value), it will be set automatically depending
    on the size of scan bounding box, so that the size of the grid in
    any dimension will not exceed 100 cells.

    Returns the list of surface triangles. It is the responsability of
    the caller to free this memory.

    WARNING: can be used, but it's still under construction. This
    function still has significant problems, most of them due the
    underlying vtk function which seems very unpredictable. I have
    tested it with scans ranging in bbox from 1m (tables) to 0.1m
    (objects) and it seems to work reasonably well. However, it always
    fails on scans with bbox on the order of 20m (rooms) regardless of
    resolution parameter.
*/
std::vector<scan_utils::Triangle>* SmartScan::createMesh(float resolution)
{
	std::vector<scan_utils::Triangle> *triangles = new std::vector<scan_utils::Triangle>;

	libTF::TFPoint bb1, bb2;
	boundingBox(bb1,bb2);
	
	float maxSize = std::max( bb2.x - bb1.x , bb2.y - bb1.y );
	maxSize = std::max(maxSize, (float)(bb2.z - bb1.z));

	if (resolution == 0) {
		resolution = maxSize / 100.0;
	}
	int sampleSize = ceil(maxSize / resolution);
	fprintf(stderr,"maxSize is %f with %d samples for %f resolution\n",maxSize, sampleSize,resolution);

	float sampleDistance;
	// the sample distance is strange... it seems not to be absolute, but rather relative to grid size
	// vtk documentation says absolutely nothing useful
	// through experimentation, a value of 0.0065 (pulled out of thin air) seems to work for a scan of 1m in size
	// so we just scale that...
	sampleDistance = 0.0065 / maxSize;

	//float cellSize = maxSize / sampleSize;
	//sampleDistance = 0.01 + 2 * cellSize;

	vtkImplicitModeller *modeller = vtkImplicitModeller::New();
	modeller->SetInput( getVtkData() );
	modeller->SetSampleDimensions(sampleSize, sampleSize, sampleSize);
	modeller->SetMaximumDistance(sampleDistance);
	modeller->SetModelBounds( bb1.x, bb2.x, bb1.y, bb2.y, bb1.z, bb2.z );

	vtkContourFilter *filter = vtkContourFilter::New();
	filter->SetInputConnection( modeller->GetOutputPort() );
	filter->SetValue(0,sampleDistance);

	vtkPolyData *output = filter->GetOutput();
	filter->Update();

	vtkCellArray *cells = output->GetPolys();
	cells->InitTraversal();
	int nPts, *pts;
	double c1[3], c2[3], c3[3];

	while( cells->GetNextCell(nPts,pts) ){
		if (nPts != 3) {
			//not a triangle
			//fprintf(stderr,"Warning: cell does not have 3 points!\n");
			continue;
		}
		output->GetPoint( pts[0], c1 );
		output->GetPoint( pts[1], c2 );
		output->GetPoint( pts[2], c3 );
		triangles->push_back( Triangle(c1, c3, c2) );
	}
	
	return triangles;
}

/*!  Return the connected components of this scan. Two components are
  considered connected if the minimum distance between them is less
  than \a thresh.

  \param minPts - The minimum number of points in a connected component for 
  it to be returned.

  Speed of implementation has been favored over speed of execution -
  multiple optimizations are possible.
 */
std::vector<SmartScan*> *SmartScan::connectedComponents(float thresh, int minPts)
{
	std::vector<SmartScan*> *result = new std::vector<SmartScan*>;
	std::list<std_msgs::Point32> currentList;

	if (!hasVtkData()) createVtkData();

	//create a temporary array where we will store all indices
	int *indices = new int[size()];

	//initialize all component indices to zero
	for (int i=0; i<size(); i++) {
		indices[i] = 0;
	}

	std_msgs::Point32 p;
	int nbr;
	vtkIdList *points = vtkIdList::New();

	int id  = 0;
	//process all the points
	for (int i=0; i<size();i++) {

		//an unassigned point
		if (indices[i] == 0) {
			id++;
			//assign a new index
			indices[i] = id;
			//process its neighbors
			currentList.push_back( getPoint(i) );
		}
		//recursively assign same index to all neighbors
		while ( !currentList.empty() ) {
			p = currentList.front();
			currentList.pop_front();

			//get all nbrs of p within thresh
			points->Reset();
			getVtkLocator()->FindPointsWithinRadius( thresh, p.x, p.y, p.z, points );

			for (int k = 0; k < points->GetNumberOfIds(); k++) {
				nbr = points->GetId(k);
				assert(nbr >= 0 && nbr < mNumPoints);

				if ( indices[nbr] == 0 ) {
					//an unassigned neighbor
					indices[nbr]=id;
					currentList.push_back(getPoint(nbr));
				} else if ( indices[nbr] != id ) {
					//a nbr already assigned to somebody else ?!?
					fprintf(stderr,"Wrong index!\n");
				}
			}
		}
	}

	//we now have the connected indices. Let's separate the scan into new scans
	int maxId = id;
	fprintf(stderr,"Found %d connected components. Copying...\n",maxId);

	int nPoints, totalPoints = 0;
	std_msgs::Point32 *scanPoints = NULL;
	//we do multiple passes because we don't really care about performance right now
	for (id = 1; id <= maxId; id++) {
		//first we count how many points there are in this component
		nPoints = 0;
		for (int i=0; i<size(); i++) {
			if (indices[i]==id) nPoints++;
		}
		//if fewer than minPts, skip
		if(nPoints < minPts) {
		  continue;
		}

		//then we allocate memory
		scanPoints = new std_msgs::Point32[nPoints];
		//then we copy the points
		int count = 0;
		for (int i=0; i<size(); i++) {
			if (indices[i]==id) {
				assert(count < nPoints);
				scanPoints[count] = getPoint(i);
				count++;
			}
		}
		//then we create the scan
		SmartScan *scan = new SmartScan();
		scan->setPoints(nPoints, scanPoints);
		result->push_back(scan);
		//and free the memory
		delete [] scanPoints;
		//and add to the sanity check
		totalPoints += nPoints;
	}

	points->Delete();
	delete [] indices;
	fprintf(stderr,"Started with %d and finished with %d points\n",size(),totalPoints);
	return result;
}

/*!  Subtracts scan \a target from this one. More specifically, any
  point in this scan is removed if there exists a point in \a target
  which is closer to it than \a thresh.
*/
void SmartScan::subtractScan(const SmartScan *target, float thresh)
{
	if (!hasVtkData()) createVtkData();

	// let's try another approach: keep track of points we keep in a separate array
	char *indices = new char[size()];
	vtkIdList *points = vtkIdList::New();

	for (int i=0; i<size(); i++) {
		// start by marking all poins as if we will keep them
		indices[i] = 1;
	}

	int keptPoints = size();
	for (int i=0; i<target->size(); i++) {
	  
	  //a point from the target
	  std_msgs::Point32 p = target->getPoint(i);
	  //find all neighbors in this scan
	  points->Reset();
	  getVtkLocator()->FindPointsWithinRadius( thresh, p.x, p.y, p.z, points );
	  for (int k = 0; k < points->GetNumberOfIds(); k++) {
	    //mark that we don't want to keep them
	    int nbr = points->GetId(k);
	    if (indices[nbr]==1) keptPoints--;
	    indices[nbr] = 0;
	  }
	}

	fprintf(stderr,"Keeping %d points\n",keptPoints);
	assert(keptPoints>=0);

	std_msgs::Point32 *newPoints = new std_msgs::Point32[keptPoints];
	int count = 0;
	for (int i=0; i<size(); i++) {
		if (indices[i]==1) {
			newPoints[count] = getPoint(i);
			count++;
		}
	}
	assert(count==keptPoints);
	setPoints(keptPoints, newPoints);

	points->Delete();
	delete [] indices;
}


/*! 
  The choice of \a support and \a pixelsPerMeter determines how many pixels are in the spin image.  \a si is filled with the relevant data.  

  This does NOT yet do any point cloud conditioning such as removing grazing points.

  \param si A Grid2D which will hold the spin image data.
  \param x, y, z The location of the spin image center.
  \param support The support of the spin image, i.e. the "radius" it cares about.
  \param pixelsPerMeter Discretization of the spin image.
  \param up "z" if you want the beta axis to be parallel to z (i.e. standard ROS coordinates); "-y" if you want smallv mode where beta points along -y.

*/
bool SmartScan::computeSpinImageFixedOrientation(Grid2D &si, float x, float y, float z, float support, float pixelsPerMeter, string up) {
        bool z_up;
	if(up.compare("z") == 0) {
	        z_up = true;
	}
	else if(up.compare("-y") == 0) {
	        z_up = false;
	}
	else {
	        cerr << "parameter 'up' is not a valid string." << endl;
	        return false;
	}

        // -- Make sure si has the appropriate dimensions.
        int height, width, hsi, wsi;
	height = ceil(2*support*pixelsPerMeter);
	width = ceil(support*pixelsPerMeter);
	si.getSize(hsi, wsi);
	assert(height==hsi && width==wsi);

	// -- Get a first cut at points close enough to care about.
	std::vector<std_msgs::Point32> *point;
	point = getPointsWithinRadius(x, y, z, support*sqrt(2));
	if(point->size() == 0) {
   	        cerr << "Not enough points at "  << x << " " << y << " " << z  << endl;
    	        return false;
	}
	
	// -- TODO: Cut away grazing points.  Will this still work when used for natural spin images? (the points are pre-rotated)

	// -- Add points within the support to the appropriate cell in the spin image.
	float beta;
	float alpha;
	for (unsigned int i=0; i<point->size(); i++) {
	  	std_msgs::Point32 &pt = (*point)[i];
		if(z_up) {
		        beta = -(pt.z - z - support);
		}
		else {
		        beta = pt.y - y + support;
		}
		if(beta >= 2*support || beta < 0)
		        continue;

		if(z_up) {
		        alpha = sqrt(pow(x - pt.x, 2) + pow(y - pt.y, 2));
		}
		else {
		        alpha = sqrt(pow(x - pt.x, 2) + pow(z - pt.z, 2));
		}

		if(alpha >= support)
		        continue;
		assert(beta >= 0);
		
		int n1, n2;
		n1 = (int)floor(beta*pixelsPerMeter);
		n2 = (int)floor(alpha*pixelsPerMeter);

		si.addElement(n1, n2, 1);
	}
	delete point;
	return true;
}

/*! 
  The choice of \a support and \a pixelsPerMeter determines how many pixels are in the spin image.  \a si is filled with the relevant data.  If the surface normal is not reliable, then this returns false.

  This does NOT yet do any point cloud conditioning such as removing grazing points.

  \param si A Grid2D which will hold the spin image data.
  \param x, y, z The location of the spin image center.
  \param support The support of the spin image, i.e. the "radius" it cares about.
  \param pixelsPerMeter Discretization of the spin image.
  \param radius Parameter passed to computePointNormal.
  \param nbrs Parameter passed to computePointNormal.

*/
bool SmartScan::computeSpinImageNatural(scan_utils::Grid2D &si, float x, float y, float z, float support, float pixelsPerMeter, float radius, int nbrs) {

        // -- Get the surface normal.
        std_msgs::Point32 normal = computePointNormal(x, y, z, radius, nbrs);
	if(normal.x == 0 && normal.y == 0 && normal.z == 0) {
	        return false;
	}
	
	// -- Setup libTF.
	libTF::TFVector tfn0, tfn1, tfn2;
	libTF::TransformReference tr;
	tfn0.frame = "FRAMEID_SPIN_ORIG";
	tfn0.time = 0; 
	tfn0.x = normal.x;
	tfn0.y = normal.y;
	tfn0.z = normal.z;

	// -- Find the transformation that makes the surface normal point up along z.
	double pitch = atan2(normal.x, normal.z);
	tr.setWithEulers("FRAMEID_SPIN_ORIG", "FRAMEID_SPIN_STAGE1", 0.0, 0.0, 0.0, 0.0, -pitch, 0.0, (libTF::TransformReference::ULLtime)0);
	tfn1 = tr.transformVector("FRAMEID_SPIN_STAGE1", tfn0);
	float roll = atan2(tfn1.y, tfn1.z);
	tr.setWithEulers("FRAMEID_SPIN_STAGE1", "FRAMEID_SPIN_STAGE2", 0.0, 0.0, 0.0, 0.0, 0.0, roll, (libTF::TransformReference::ULLtime)0);
	tfn2 = tr.transformVector("FRAMEID_SPIN_STAGE2", tfn1);

	// -- Transform the spin image center point.
	libTF::TFPoint center0, center2;
	center0.frame = "FRAMEID_SPIN_ORIG";
	center0.time = 0;
	center0.x = x;
	center0.y = y;
	center0.z = z;
	center2 = tr.transformPoint("FRAMEID_SPIN_STAGE2", center0);

	// -- Transform the nearby points.
	//ss.applyTransform(tr.getMatrix("FRAMEID_SPIN_ORIG", "FRAMEID_SPIN_STAGE2", 0));  Why doesn't this work? -- ss was a smartscan of cld.
	std_msgs::PointCloud* cld = getPointsWithinRadiusPointCloud(x, y, z, support*sqrt(2));
	libTF::TFPoint ptf, ptf2;
	for (unsigned int j=0; j<cld->get_pts_size(); j++) {
	        ptf.frame = "FRAMEID_SPIN_ORIG";
		ptf.time = 0;
		ptf.x = cld->pts[j].x;
		ptf.y = cld->pts[j].y;
		ptf.z = cld->pts[j].z;
		ptf2 = tr.transformPoint("FRAMEID_SPIN_STAGE2", ptf);
		cld->pts[j].x = ptf2.x;
		cld->pts[j].y = ptf2.y;
		cld->pts[j].z = ptf2.z;
	}

	// -- Compute the spin image.
	SmartScan ss;
	ss.setFromRosCloud(*cld);
	return ss.computeSpinImageFixedOrientation(si, center2.x, center2.y, center2.z, support, pixelsPerMeter, string("z"));
}


template <typename T>
void SmartScan::insertInOctree(Octree<T> *o, T value)
{
	std_msgs::Point32 p;
	for(int i=0; i<size(); i++) {
		p = getPoint(i);
		o->insert(p.x, p.y, p.z, value);
	}
}

/* tell the compiler to instantiate some possible forms of this
   call. This is really unfortunate, but no other solution seemed
   preferable. Other solutions include: including octree.h (which is
   huge) in the smartScan.h header, or including smartScan.h in the
   octree.h header (which is already huge) and making this member of
   the Octree class instead of the SmartScan class*/

template void SmartScan::insertInOctree<int>(Octree<int>*,int);
template void SmartScan::insertInOctree<char>(Octree<char>*,char);
template void SmartScan::insertInOctree<unsigned char>(Octree<unsigned char>*,unsigned char);
template void SmartScan::insertInOctree<unsigned int>(Octree<unsigned int>*,unsigned int);
template void SmartScan::insertInOctree<float>(Octree<float>*,float);
template void SmartScan::insertInOctree<double>(Octree<double>*,double);

