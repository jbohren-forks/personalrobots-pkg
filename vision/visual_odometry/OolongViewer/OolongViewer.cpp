/*
 * OolongViewer.cpp
 *
 *  Created on: Aug 19, 2008
 *      Author: jdchen
 */

#include "OolongViewer.h"

// VTK headers
#include <vtkPointSource.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkDataArray.h>
#include <vtkAppendPolyData.h>
#include <vtkPointSet.h>
#include <vtkCellArray.h>
#include <vtkQuad.h>
#include <vtkAxes.h>
#include <vtkTubeFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkProperty.h>
#include <vtkVRMLExporter.h>
// opencv
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string>
#include <iostream>
#include <vector>
using namespace std;

#include <boost/foreach.hpp>

#include "MyVTKLinePlotter.h"
#include "MyVTKPointPlotter.h"

#define FIRST_ONE_IS_GROUNDTRUTH_OFFSET 0

OolongViewer::OolongViewer():
  end_point_(cvPoint3D64f(0,0,0))
  {
	// TODO Auto-generated constructor stub
}

OolongViewer::~OolongViewer() {
  // ~vtkActor() is protected
  BOOST_FOREACH(vtkActor* actor, path_actors_) {
    actor->Delete();
  }
}

CvMat* OolongViewer::loadTrajectory(const char *filename){
	const char* fn;
	if (filename == NULL) {
		fn = "Data/indoor1/shifts.xml";
	} else {
		fn = filename;
	}

	CvMat* shifts = (CvMat *)cvLoad(fn);
	return shifts;
}

CvMat* OolongViewer::loadRotations(const char *filename) {
	const char*fn;
	if (filename == NULL){
		fn = "Data/indoor1/rods.xml";
	} else {
		fn = filename;
	}

	CvMat* rods = (CvMat *)cvLoad(fn);
	return rods;
}

CvMat* OolongViewer::loadFramePoses(const string& filename) {
	CvMat* framePoses = (CvMat *)cvLoad(filename.c_str());
	return framePoses;
}

CvMat* OolongViewer::loadPointCloud(const string& filename) {
	CvMat* pointCloud = (CvMat *)cvLoad(filename.c_str());
	return pointCloud;
}

void OolongViewer::viewTrajectory(CvMat& trajectory){

}

static double colorMap[][3] = {
		{255, 255, 255},  // white
		{255, 0, 0},	    // red
		{255, 255, 0},    // yellow
		{0, 255, 0},      // green
		{0, 255, 255},    // cyan
		{255, 0, 255},     // magenta
    {0, 0, 255}      // blue
};
static int colorMapSize = sizeof(colorMap)/3;

void OolongViewer::view(const int numDirs, const char *dirnames[]){
  for (int i=0; i<numDirs; i++) {
    view(dirnames[i], i);
  }
#if 0 // 0: for no planes
  //
  // Draw a 2D quad on X-Z plane
  //
  vtkPoints* quadPoints = vtkPoints::New();
  quadPoints->SetNumberOfPoints(4);
  quadPoints->InsertPoint(0, 0, 0, 0);
  quadPoints->InsertPoint(1, -7.5, 0, 0);
  quadPoints->InsertPoint(2, -7.5, 0, 4.);
  quadPoints->InsertPoint(3, 0, 0, 4.);
  vtkQuad *aQuad = vtkQuad::New();
  aQuad->GetPointIds()->SetId(0, 0);
  aQuad->GetPointIds()->SetId(1, 1);
  aQuad->GetPointIds()->SetId(2, 2);
  aQuad->GetPointIds()->SetId(3, 3);
  vtkUnstructuredGrid *aQuadGrid = vtkUnstructuredGrid::New();
  aQuadGrid->Allocate(1, 1);
  aQuadGrid->InsertNextCell(aQuad->GetCellType(), aQuad->GetPointIds());
  aQuadGrid->SetPoints(quadPoints);
  vtkDataSetMapper *aQuadMapper = vtkDataSetMapper::New();
  aQuadMapper->SetInput(aQuadGrid);
  vtkActor *aQuadActor = vtkActor::New();
  aQuadActor->SetMapper(aQuadMapper);
  aQuadActor->AddPosition(0, 0, 0);
  aQuadActor->GetProperty()->SetDiffuseColor(1, 1, 1);
  aQuadActor->GetProperty()->SetOpacity(.5);

  //
  // Draw a 2D Quad on X-Y plane
  vtkPoints* quadPoints2 = vtkPoints::New();
  quadPoints2->SetNumberOfPoints(4);
  quadPoints2->InsertPoint(0,    0,   0, 0);
  quadPoints2->InsertPoint(1, -7.5,   0, 0);
  quadPoints2->InsertPoint(2, -7.5, -.2, 0);
  quadPoints2->InsertPoint(3,   0., -.2, 0);
  vtkQuad *aQuad2 = vtkQuad::New();
  aQuad2->GetPointIds()->SetId(0, 0);
  aQuad2->GetPointIds()->SetId(1, 1);
  aQuad2->GetPointIds()->SetId(2, 2);
  aQuad2->GetPointIds()->SetId(3, 3);
  vtkUnstructuredGrid *aQuadGrid2 = vtkUnstructuredGrid::New();
  aQuadGrid2->Allocate(1, 1);
  aQuadGrid2->InsertNextCell(aQuad2->GetCellType(), aQuad2->GetPointIds());
  aQuadGrid2->SetPoints(quadPoints2);
  vtkDataSetMapper *aQuadMapper2 = vtkDataSetMapper::New();
  aQuadMapper2->SetInput(aQuadGrid2);
  vtkActor *aQuadActor2 = vtkActor::New();
  aQuadActor2->SetMapper(aQuadMapper2);
  aQuadActor2->AddPosition(0, 0, 0);
  aQuadActor2->GetProperty()->SetDiffuseColor(1, 1, 1);
  aQuadActor2->GetProperty()->SetOpacity(.25);
#endif

  // create axes
//  popSplatter->Update();
//  popSplatter->GetOutput()->GetBounds(bounds);
  vtkAxes *axes = vtkAxes::New();
//    axes->SetOrigin(bounds[0], bounds[2], bounds[4]);
//    axes->SetScaleFactor(popSplatter->GetOutput()->GetLength()/5);
  vtkTubeFilter *axesTubes = vtkTubeFilter::New();
    axesTubes->SetInputConnection(axes->GetOutputPort());
    axesTubes->SetRadius(axes->GetScaleFactor()/100.0);
    axesTubes->SetNumberOfSides(6);
  vtkPolyDataMapper *axesMapper = vtkPolyDataMapper::New();
    axesMapper->SetInputConnection(axesTubes->GetOutputPort());
  vtkActor *axesActor = vtkActor::New();
    axesActor->SetMapper(axesMapper);

  //
  // Create the Renderer and assign actors to it. A renderer is like a
  // viewport. It is part or all of a window on the screen and it is
  // responsible for drawing the actors it has.  We also set the background
  // color here.
  //
  vtkRenderer *ren1= vtkRenderer::New();
  BOOST_FOREACH(vtkActor* actor, path_actors_) {
    ren1->AddActor( actor );
  }
#if 0
  ren1->AddActor( aQuadActor);
  ren1->AddActor( aQuadActor2);
#endif
  ren1->AddActor( axesActor);


//  ren1->SetBackground( 0.05, 0.1, 0.2 );
  ren1->SetBackground( 0.1, 0.2, 0.4 );

  //
  // Finally we create the render window which will show up on the screen.
  // We put our renderer into the render window using AddRenderer. We also
  // set the size to be 300 pixels by 300.
  //
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer( ren1 );
  renWin->SetSize( 640, 480 );

  //
  // The vtkRenderWindowInteractor class watches for events (e.g., keypress,
  // mouse) in the vtkRenderWindow. These events are translated into
  // event invocations that VTK understands (see VTK/Common/vtkCommand.h
  // for all events that VTK processes). Then observers of these VTK
  // events can process them as appropriate.
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  //
  // By default the vtkRenderWindowInteractor instantiates an instance
  // of vtkInteractorStyle. vtkInteractorStyle translates a set of events
  // it observes into operations on the camera, actors, and/or properties
  // in the vtkRenderWindow associated with the vtkRenderWinodwInteractor.
  // Here we specify a particular interactor style.
  vtkInteractorStyleTrackballCamera *style =
    vtkInteractorStyleTrackballCamera::New();
  iren->SetInteractorStyle(style);

  //
  // Unlike the previous scripts where we performed some operations and then
  // exited, here we leave an event loop running. The user can use the mouse
  // and keyboard to perform the operations on the scene according to the
  // current interaction style. When the user presses the "e" key, by default
  // an ExitEvent is invoked by the vtkRenderWindowInteractor which is caught
  // and drops out of the event loop (triggered by the Start() method that
  // follows.
  //
  iren->Initialize();
  iren->Start();

  vtkVRMLExporter* vrml = vtkVRMLExporter::New();
  vrml->SetInput(renWin);
  vrml->SetFileName("paths.wrl");
  vrml->Write();

  //
  // Final note: recall that an observers can watch for particular events and
  // take appropriate action. Pressing "u" in the render window causes the
  // vtkRenderWindowInteractor to invoke a UserEvent. This can be caught to
  // popup a GUI, etc. So the Tcl Cone5.tcl example for an idea of how this
  // works.

  //
  // Free up any objects we created. All instances in VTK are deleted by
  // using the Delete() method.
  //
// TODO: move the following 3 to appropriate places
//  appPolyData->Delete();
//  trajectoryDataMapper->Delete();
//  trajectorActor->Delete();
  ren1->Delete();
  vrml->Delete();
  renWin->Delete();
  iren->Delete();
  style->Delete();
}

void OolongViewer::view(const char dirname[], int path_id) {
  string framePosesFile;
  if (dirname == NULL) {
		framePosesFile = "Data/indoor1";
	} else {
	  framePosesFile = dirname;
	}
  framePosesFile += "/framePoses.xml";
#if FIRST_ONE_IS_GROUNDTRUTH_OFFSET==1
  if (path_id == 0) {
    printf("%s, \"Ground Truth\"\n", dirname);
  } else {
    printf("%s\n", dirname);
  }
#else
  printf("%s\n", dirname);
#endif

  CvMat* framePoses = loadFramePoses(framePosesFile);
#if 0 // old code the read old format.
	string shiftfile(dirname);
	string rodsfile(dirname);
	shiftfile.append("/shifts.xml");
	rodsfile.append("/rods.xml");
	CvMat* trajectory = loadTrajectory(shiftfile.c_str());
	CvMat* rods       = loadRotations(rodsfile.c_str());
#endif

	CvMat _rod;
	CvMat _trajectory;
	CvMat* rods  = &_rod;
	CvMat* trajectory = &_trajectory;

	switch(framePoses->cols) {
	case 7:
    cvGetSubRect(framePoses, &_rod,        cvRect(1, 0, 3, framePoses->rows));
	  cvGetSubRect(framePoses, &_trajectory, cvRect(4, 0, 3, framePoses->rows));
	  break;
	case 3:
    cvGetSubRect(framePoses, &_trajectory,        cvRect(0, 0, 3, framePoses->rows));
    _rod.rows=0;
    _rod.cols=0;
    _rod.data.ptr=NULL;
	  break;
	default:
	  cerr << "unsupported format: framePoses->cols="<<framePoses->cols;
	}

	int numFrames = trajectory->rows;
	int numRods   = rods->rows;
	if (numRods != numFrames && framePoses->cols != 3) {
	  // for old format
		cerr << "num of enties in rods.xml and shifts.xml do not match"<<endl;
	}
  assert(numRods == numFrames||framePoses->cols == 3);
	vtkPoints *points = vtkPoints::New();
	vtkCellArray * ca = vtkCellArray::New();

//	cout <<"Inserting points"<<endl;
	double x0=0, y0=0, z0=0;
	double _rot[9];
	CvMat rot = cvMat(3, 3, CV_64FC1, _rot);
	double _z_axis[] = {0, 0, 1};
	const CvMat zAxis = cvMat(1, 1, CV_64FC3, _z_axis);
	double _z_axis_rotated[3];
	CvMat zAxisRotated = cvMat(1, 1, CV_64FC3, _z_axis_rotated);

	MyVTKLinePlotter linePlotter;
	MyVTKPointPlotter pointPlotter;
	pointPlotter.SetPointRadius(10);

	CvMat rod;
	double normal_scale=.1;
  double distScale = 1./1000.;
  /// @todo temporary stuff when we switching from mm to meter
  if (path_id == 0) {
    distScale = 1.0;
  }
	int colorIndex = 0;
	int start = 0;
	int end   = numFrames;

//	start = 1300;
//	end   = 1400;

	int numKeyFrames = 0;
	int maxNumKeyFrames = numFrames;
	double dist = 0;
	int frameIndexBase = 0;
//	frameIndexBase =1300;
	for (int i=start; i<end; i++) {
		int fi = cvmGet(framePoses, i,0);
		double x = cvmGet(trajectory, i, 0)*distScale;
		double y = cvmGet(trajectory, i, 1)*distScale;
		double z = cvmGet(trajectory, i, 2)*distScale;


		if (i==start && rods->cols==3) {
		  // drawing the camera direction
			cvGetRow(rods, &rod, i);
			cvRodrigues2(&rod, &rot);
			// rotate vector (0, 0, 1) by the rotation matrix and then draw it
			cvTransform(&zAxis, &zAxisRotated, &rot, NULL);
			// draw an arrow from (x, y, z) to (x, y, z)+zAxisRotated
			cvScale(&zAxisRotated, &zAxisRotated, normal_scale);
			linePlotter.PlotLine(x, y, z, x+_z_axis_rotated[0], y+_z_axis_rotated[1], z+_z_axis_rotated[2], 1.0);
//			printf("%4d, %8.4f, %8.4f, %8.4f, %8.4f\n", fi, x, y, z, dist);
			numKeyFrames++;
			points->InsertNextPoint(x, y, z);
			x0=x;y0=y;z0=z;
		}

		if ((fabs(x) < .0000001 && fabs(y)< .0000001 && fabs(z)<.0000001)) {
			// skip
		} else {
			if (numKeyFrames > 0) {
				double dx = x-x0;
				double dy = y-y0;
				double dz = z-z0;
				dist += sqrt(dx*dx+dy*dy+dz*dz);
			}

			if (i==end-1) // last point
			{
#if FIRST_ONE_IS_GROUNDTRUTH_OFFSET==1
			  if (path_id == 0) {
			    // the "ground truth for end point"
			    end_point_.x = x;
			    end_point_.y = y;
			    end_point_.z = z;
			  }
#endif
			  double dx = x - end_point_.x;
			  double dy = y - end_point_.y;
			  double dz = z - end_point_.z;
			  double err = sqrt(dx*dx + dy*dy + dz*dz);

			  printf("%4d, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f\n", fi, x, y, z, dist, err);
			}
			numKeyFrames++;
			points->InsertNextPoint(x, y, z);

			if (rods->cols==3) {
			  // draw a line around the z axis of current frame (to which direction
			  // the camera is pointing to
			  cvGetRow(rods, &rod, i);
			  cvRodrigues2(&rod, &rot);
			  // rotate vector (0, 0, 1) by the rotation matrix and then draw it
			  cvTransform(&zAxis, &zAxisRotated, &rot, NULL);
			  // draw an arrow from (x, y, z) to (x, y, z)+zAxisRotated
			  cvScale(&zAxisRotated, &zAxisRotated, normal_scale);
			  linePlotter.PlotLine(x, y, z, x+_z_axis_rotated[0], y+_z_axis_rotated[1], z+_z_axis_rotated[2], 1.0);
			}
			// draw the inliers of current frame
			if (numKeyFrames <= maxNumKeyFrames) {
				string pointCloudFile(dirname);
				char fn[256];
				int frameIndex = cvmGet(framePoses, i+frameIndexBase,0);
				sprintf(fn, "/inliers1_%04d.xml", frameIndex);
				pointCloudFile.append(fn);
				CvMat *pointCloud = loadPointCloud(pointCloudFile);
				if (pointCloud) {
				  double *color = colorMap[colorIndex];
				  for (int j=0; j<pointCloud->rows; j++) {
				    CvScalar pt = cvGet2D(pointCloud, j, 0);

				    // draw points that are not too far away
				    if (pt.val[0] < 20000 && pt.val[0] > -8000 &&
				        pt.val[1] < 4000 && pt.val[1] > -4000 &&
				        pt.val[2] < 20000 && pt.val[2] > -2000 ) {

				      pointPlotter.PlotPoint(pt.val[0], pt.val[1], pt.val[2], color[0], color[1], color[2]);
				    }
				  }
				  colorIndex++;
				  colorIndex %= colorMapSize;
				}
			}


			// save for next point
			x0 = x;
			y0 = y;
			z0 = z;
		}
	}
	ca->InsertNextCell(numKeyFrames);
	for (int i=0; i<numKeyFrames; i++) {
		ca->InsertCellPoint(i);
	}

	cout << flush;

	vtkPolyData* trajectoryData = vtkPolyData::New();
	trajectoryData->SetPoints(points);
	trajectoryData->SetLines(ca);

	vtkAppendPolyData *appPolyData = vtkAppendPolyData::New();
	appPolyData->AddInput(trajectoryData);

	//
	// In this example we terminate the pipeline with a mapper process object.
	// (Intermediate filters such as vtkShrinkPolyData could be inserted in
	// between the source and the mapper.)  We create an instance of
	// vtkPolyDataMapper to map the polygonal data into graphics primitives. We
	// connect the output of the cone source to the input of this mapper.
	//
	vtkPolyDataMapper *trajectoryDataMapper = vtkPolyDataMapper::New();
	trajectoryDataMapper->SetInput(trajectoryData);
	trajectoryDataMapper->SetColorModeToMapScalars();

//	coneMapper->SetInputConnection( appPolyData->GetOutputPort() );

	//
	// Create an actor to represent the cone. The actor orchestrates rendering
	// of the mapper's graphics primitives. An actor also refers to properties
	// via a vtkProperty instance, and includes an internal transformation
	// matrix. We set this actor's mapper to be coneMapper which we created
	// above.
	//
	vtkActor *trajectoryActor = vtkActor::New();
	trajectoryActor->SetMapper( trajectoryDataMapper );
	int color_index = path_id % colorMapSize;
//  trajectoryActor->GetProperty()->SetDiffuseColor(colorMap[color_index][0], colorMap[color_index][1], colorMap[color_index][2]);
  trajectoryActor->GetProperty()->SetColor(colorMap[color_index]);
  trajectoryActor->GetProperty()->SetEdgeColor(colorMap[color_index]);

	path_actors_.push_back(trajectoryActor);
	path_actors_.push_back( linePlotter.CreateActor() );
	path_actors_.push_back( pointPlotter.CreateActor() );
}

int main(const int argc, const char *argv[]) {
	OolongViewer ov;
//	ov.display3d();
	if (argc==1) {
		ov.view(0);
	} else {
		ov.view(argc-1, &argv[1]);
	}
}
