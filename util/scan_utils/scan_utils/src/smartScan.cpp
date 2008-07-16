#include "smartScan.h"

#include "math.h"
#include <list>

#include "vtkFloatArray.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkUnstructuredGrid.h"
#include "vtkDelaunay2D.h"
#include "vtkDelaunay3D.h"
#include "vtkCellArray.h"
#include "vtkPointLocator.h"
#include "vtkIterativeClosestPointTransform.h"
#include "vtkMatrix4x4.h"
#include "vtkCellLocator.h"
#include "vtkLandmarkTransform.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTransformFilter.h"
#include "vtkImplicitModeller.h"
#include "vtkPolyDataMapper.h"
#include "vtkContourFilter.h"
#include "vtkSurfaceReconstructionFilter.h"
#include "vtkMarchingCubes.h"


#include "newmat10/newmatap.h"
#include <iomanip>
#include "newmat10/newmatio.h"

using namespace scan_utils;

#define TFRT 1
#define TFMF 2

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
	mScannerPos.frame = TFMF; mScannerPos.time = 0;
	//should normalize them too...
	mScannerDir.x = dx; mScannerDir.y = dy; mScannerDir.z = dz;
	mScannerDir.frame = TFMF; mScannerDir.time = 0;
	mScannerUp.x = ux; mScannerUp.y = uy; mScannerUp.z = uz;
	mScannerUp.frame = TFMF; mScannerUp.time = 0;
}

void SmartScan::getScanner(float &px, float &py, float &pz, 
			   float &dx, float &dy, float &dz,
			   float &ux, float &uy, float &uz)
{
	px = mScannerPos.x; py = mScannerPos.y; pz = mScannerPos.z;
	dx = mScannerDir.x; dy = mScannerDir.y; dz = mScannerDir.z;
	ux = mScannerUp.x; uy = mScannerUp.y; uz = mScannerUp.z;
}

void SmartScan::setPoints(int numPoints, const std_msgs::Point3DFloat32 *points)
{
	// for now we are making explicit copies, but later this will be changed to not
	// copy the data but just store it
	clearData();
	mNumPoints = numPoints;
	mNativePoints = new std_msgs::Point3DFloat32[mNumPoints];
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
	mNativePoints = new std_msgs::Point3DFloat32[mNumPoints];
	for (int i=0; i<mNumPoints; i++) {
		mNativePoints[i].x = points[3*i+0];
		mNativePoints[i].y = points[3*i+1];
		mNativePoints[i].z = points[3*i+2];
	}
}


std_msgs::Point3DFloat32 SmartScan::transformPoint(const std_msgs::Point3DFloat32 &p_in,
						   libTF::TransformReference &tr) const
{

	libTF::TFPoint p;
	p.x = p_in.x; p.y = p_in.y; p.z = p_in.z;
	p.frame = TFMF;	p.time = 0;
	p = tr.transformPoint(TFRT, p);

	std_msgs::Point3DFloat32 p_out;
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
	tr.setWithMatrix( TFMF, TFRT, M, 0);

	for (int i=0; i<mNumPoints; i++) {
		mNativePoints[i] = transformPoint( mNativePoints[i], tr );
	}

	mScannerPos = tr.transformPoint(TFRT, mScannerPos);
	mScannerDir = tr.transformVector(TFRT, mScannerDir);
	mScannerUp = tr.transformVector(TFRT, mScannerUp);
	mScannerPos.frame = TFMF; mScannerPos.time = 0;
	mScannerDir.frame = TFMF; mScannerDir.time = 0;
	mScannerUp.frame = TFMF; mScannerUp.time = 0;

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
	std_msgs::Point3DFloat32 p;
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
	mNativePoints =  new std_msgs::Point3DFloat32[mNumPoints];

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

/*! Adds the points in the scan \a target to this one. Does not check
    for duplicate points.
 */
void SmartScan::addScan(const SmartScan *target)
{
	if ( !target->size() ) return;

	std_msgs::Point3DFloat32 *newPoints = new std_msgs::Point3DFloat32[mNumPoints + target->size()];

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

    \param alpha No points that are further apart than this values
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
	std_msgs::Point3DFloat32 *newPoints,p;
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
	newPoints = new std_msgs::Point3DFloat32[numNewPoints];
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
	std_msgs::Point3DFloat32 p;
	if (! hasVtkData() ) createVtkData();
	vtkIdList *result = vtkIdList::New();

	// allocate memory as if we will keep all points, but later we'll do a copy and a delete
	std_msgs::Point3DFloat32 *newPoints = new std_msgs::Point3DFloat32[mNumPoints];
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
  points. For example, if \param threshold = 10 all points whose
  normal is closer than 10 degrees to beeing perpendicular to the
  scanner direction are removed.

  \param removeOutliers What do we do with points that have too few
  neighbors to compute normals: if this flag is true, these points are
  removed (as in removeOutliers(...) )

  For computing point normals we look for at least \a nbrs neighbors
  within a sphere of radius \a radius.
 */
void SmartScan::removeGrazingPoints(float threshold, bool removeOutliers, float radius, int nbrs)
{
	// allocate memory as if we will keep all points, but later we'll do a copy and a delete
	std_msgs::Point3DFloat32 *newPoints = new std_msgs::Point3DFloat32[mNumPoints];
	int numNewPoints = 0;

	//we get the threshold in degrees
	threshold = fabs( threshold * M_PI / 180.0 );
	threshold = fabs( cos(M_PI / 2 - threshold) );

	std_msgs::Point3DFloat32 scanner;
	scanner.x = mScannerPos.x; 
	scanner.y = mScannerPos.y; 
	scanner.z = mScannerPos.z;

	for (int i=0; i<mNumPoints; i++) {
		std_msgs::Point3DFloat32 normal = computePointNormal(i,radius,nbrs);
		//check for outliers
		if ( norm(normal) < 0.5 && removeOutliers) continue;
	
		//compute direction to scanner
		std_msgs::Point3DFloat32 scannerDirection;
		std_msgs::Point3DFloat32 p = getPoint(i);
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
*/
void SmartScan::normalHistogramPlane(std_msgs::Point3DFloat32 &planePoint, std_msgs::Point3DFloat32 &planeNormal,
				     float radius, int nbrs)
{
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
		std_msgs::Point3DFloat32 normal = computePointNormal(i,radius,nbrs);
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
		std_msgs::Point3DFloat32 normal = computePointNormal(i,radius,nbrs);
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
}

/*! Computes the dominant plane in the scan using RANSAC.

  \param planePoint, planeNormal Output values, holding the plane found by the function

  \param iterations Number of RANSAC iterations to be performed

  \param distThresh Internal RANSAC threshold: points that are closer
  than this value to a hypothesis plane are considered inliers

 */
void SmartScan::ransacPlane(std_msgs::Point3DFloat32 &planePoint, std_msgs::Point3DFloat32 &planeNormal,
			    int iterations, float distThresh)
{
	int selPoints = 3;

	//if this is true, each plane is re-fit to all inliers after inliers are calulated
	//otehrwise, planes are just fit to the initial random chosen points.
	bool refitToConsensus = true;

	int it = 0, consensus, maxConsensus = 0;
	int i,k;
	float dist;

	NEWMAT::Matrix M(selPoints,3);
	std_msgs::Point3DFloat32 mean, consMean, normal, dif, p;
	std_msgs::Point3DFloat32 zero; zero.x = zero.y = zero.z = 0.0;
	std::list<std_msgs::Point3DFloat32> consList;

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
		normal = SVDPlaneNormal(&M, selPoints);

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
				std::list<std_msgs::Point3DFloat32>::iterator it;
				for (k=0, it = consList.begin(); it!=consList.end(); it++, k++) {
					CM->element(k,0) = (*it).x - consMean.x;
					CM->element(k,1) = (*it).y - consMean.y;
					CM->element(k,2) = (*it).z - consMean.z;
				}
				planeNormal = SVDPlaneNormal(CM,consensus);
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
}

/*!  Removes all points that are closer than \a thresh to the plane
  defined by \a planePoint and \a planeNormal
 */
void SmartScan::removePlane(const std_msgs::Point3DFloat32 &planePoint, 
			    const std_msgs::Point3DFloat32 &planeNormal, float thresh)
{
	//remove points that are close to plane
	std_msgs::Point3DFloat32 *newPoints = new std_msgs::Point3DFloat32[mNumPoints];
	std_msgs::Point3DFloat32 dif, p;
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
  std::vector<Point3DFloat32>* - it is the responsability of the
  caller to free this memory.
*/
std::vector<std_msgs::Point3DFloat32>* SmartScan::getPointsWithinRadius(float x, float y, float z, float radius)
{
	std::vector<std_msgs::Point3DFloat32> *resPts = new std::vector<std_msgs::Point3DFloat32>;
	vtkIdList *result = vtkIdList::New();

	getVtkLocator()->FindPointsWithinRadius( radius, x, y, z, result );
	int n = result->GetNumberOfIds();
	for (int i=0; i<n; i++){
		int nbrId = result->GetId(i);
		std_msgs::Point3DFloat32 p = getPoint(nbrId);
		resPts->push_back(p);
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
std_msgs::Point3DFloat32 SmartScan::computePointNormal(int id, float radius, int nbrs)
{
	// radius - how large is the radius in which we look for nbrs for computing normal
	// nbrs - min number of nbrs we use for normal computation
	assert(id >=0 && id < mNumPoints);
	int nbrId;

	vtkIdList *result = vtkIdList::New();
	std_msgs::Point3DFloat32 zero; zero.x = zero.y = zero.z = 0.0;
	std_msgs::Point3DFloat32 p = getPoint(id);

	getVtkLocator()->FindPointsWithinRadius( radius, p.x, p.y, p.z, result );
	//we don't have enough nbrs for a reliable normal
	int n = result->GetNumberOfIds();
	if ( n < nbrs ) {
		result->Delete();
		return zero;
	}

	//find the mean point for normalization
	std_msgs::Point3DFloat32 mean = zero;
	for (int i=0; i<n; i++) {
		nbrId = result->GetId(i);
		assert(nbrId >= 0 && nbrId < mNumPoints);
		p = getPoint(nbrId);

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
		p = getPoint(nbrId);

		M.element(i,0) = p.x - mean.x;
		M.element(i,1) = p.y - mean.y;
		M.element(i,2) = p.z - mean.z;
	}
	result->Delete();

	return SVDPlaneNormal(&M,n);
}

/*! Given an \a n by 3 matrix \a M in NEWMAT format, performs
  SVD and returns the direction of the least significant singular
  vector. If the matrix is a list of vertices, this corresponds to the
  direction of the normal of a plane fit to those points. Vertices are
  expected to be normalized first (of mean 0).
*/
std_msgs::Point3DFloat32 SmartScan::SVDPlaneNormal(NEWMAT::Matrix *M, int n)
{
	NEWMAT::Matrix U(n,3);
	NEWMAT::DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	SVD(*M, D, U, V);

	std_msgs::Point3DFloat32 normal;
	normal.x = V.element(0,2);
	normal.y = V.element(1,2);
	normal.z = V.element(2,2);
	//std::cout << *M << std::endl;
	//std::cout << U << std::endl;
	//std::cout << D << std::endl;
	//std::cout << V << std::endl;
	//fprintf(stderr,"%f %f %f \n",normal.x, normal.y, normal.z);
	return normal;
}

std::vector<scan_utils::Triangle>* SmartScan::createMesh()
{
	std::vector<scan_utils::Triangle> *triangles = new std::vector<scan_utils::Triangle>;

	vtkImplicitModeller *modeller = vtkImplicitModeller::New();
	modeller->SetInput( getVtkData() );
	//	modeller->SetSampleDimensions(100, 100, 100);
	modeller->SetMaximumDistance(0.2);
	//	modeller->SetModelBounds(-1,-1,-1,1,1,1);

	vtkSurfaceReconstructionFilter *surface = vtkSurfaceReconstructionFilter::New();
	surface->SetSampleSpacing(0.01);
	surface->SetInput( getVtkData() );

	vtkContourFilter *filter = vtkContourFilter::New();
	filter->SetValue(0,0.0);
	vtkMarchingCubes *cubes = vtkMarchingCubes::New();
	cubes->SetValue(0,0.0);
	vtkPolyData *output;

	/*
	output = filter->GetOutput();
	//filter->SetInputConnection( modeller->GetOutputPort() );
	filter->SetInputConnection( surface->GetOutputPort() );
	filter->Update();
	*/

	output = cubes->GetOutput();
	//cubes->SetInputConnection( modeller->GetOutputPort() );
	cubes->SetInputConnection( surface->GetOutputPort() );
	cubes->Update();

	
	fprintf(stderr,"Filter has updated and has %d cells\n", output->GetNumberOfCells() );
	
	vtkCellArray *cells = output->GetPolys();
	cells->InitTraversal();
	int nPts, *pts;
	double c1[3], c2[3], c3[3];

	while( cells->GetNextCell(nPts,pts) ){
		if (nPts != 3) {
			//not a triangle
			fprintf(stderr,"Warning: cell does not have 3 points!\n");
			continue;
		}
		output->GetPoint( pts[0], c1 );
		output->GetPoint( pts[1], c2 );
		output->GetPoint( pts[2], c3 );
		triangles->push_back( Triangle(c1, c2, c3) );
	}
	
	modeller->Delete();
	return triangles;
}

/*!  Return the connected components of this scan. Two components are
  considered connected if the minimum distance between them is less
  than \a thresh.

  Speed of implementation has been favored over speed of execution -
  multiple optimizations are possible.
 */
std::vector<SmartScan*> *SmartScan::connectedComponents(float thresh)
{
	std::vector<SmartScan*> *result = new std::vector<SmartScan*>;
	std::list<std_msgs::Point3DFloat32> currentList;

	if (!hasVtkData()) createVtkData();

	//create a temporary array where we will store all indices
	int *indices = new int[size()];

	//initialize all component indices to zero
	for (int i=0; i<size(); i++) {
		indices[i] = 0;
	}

	std_msgs::Point3DFloat32 p;
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
	std_msgs::Point3DFloat32 *scanPoints = NULL;
	//we do multiple passes because we don't really care about performance right now
	for (id = 1; id <= maxId; id++) {
		//first we count how many points there are in this component
		nPoints = 0;
		for (int i=0; i<size(); i++) {
			if (indices[i]==id) nPoints++;
		}
		//then we allocate memory
		scanPoints = new std_msgs::Point3DFloat32[nPoints];
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
