#include "MyVTKPointPlotter.h"


#include "vtkPoints.h"
#include "vtkUnsignedCharArray.h"
#include "vtkPolyData.h"
#include "vtkActor.h"
#include "vtkDataArray.h"
#include "vtkGlyph3D.h"
#include "vtkDiskSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPointData.h"
#include <vtkSphereSource.h>

#define DrawPointAsDisk

MyVTKPointPlotter::MyVTKPointPlotter()
{
	pts = NULL ;
	scalars = NULL ;

	SetPointRadius();
	SetPointResolution();

}

MyVTKPointPlotter::~MyVTKPointPlotter()
{
	if (pts)
		pts->Delete();
	if (scalars)
		scalars->Delete();
}

void MyVTKPointPlotter::PlotPoint(double x, double y, double z,
									unsigned char r, unsigned char g, unsigned char b)
{
	if (pts==NULL)
		pts = vtkPoints::New();

	if (scalars==NULL)
	{
		scalars = vtkUnsignedCharArray::New();
		scalars->SetNumberOfComponents(3);
	}

	pts->InsertNextPoint(x,y,z);
	scalars->InsertNextTuple3(r,g,b);

}

vtkPolyData* MyVTKPointPlotter::CreatePolyData()
{
#ifdef DrawPointAsDisk
	// drawing points as disks
	vtkDiskSource* src = vtkDiskSource::New();
	src->SetRadialResolution(1);
	src->SetCircumferentialResolution(pt_res);

	src->SetInnerRadius(0.0);
	src->SetOuterRadius(pt_radius);
#else
	// drawing points as spheres
	vtkSphereSource* src = vtkSphereSource::New();
	src->SetThetaResolution(pt_res);
	src->SetPhiResolution(pt_res);
	src->SetRadius(pt_radius);
#endif


	vtkPolyData* polyData = vtkPolyData::New();
	polyData->SetPoints(pts);
	polyData->GetPointData()->SetScalars(scalars);

	vtkGlyph3D* glyph = vtkGlyph3D::New();
	glyph->SetSourceConnection(src->GetOutputPort());
	glyph->SetInput(polyData);

	glyph->SetColorModeToColorByScalar();
	glyph->SetScaleModeToDataScalingOff() ;


	vtkPolyData* output = glyph->GetOutput();
	return output ;
}

vtkActor* MyVTKPointPlotter::CreateActor()
{
	vtkPolyData* polyData = CreatePolyData();

	vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
	mapper->SetInput(polyData);

	vtkActor* actor = vtkActor::New();
	actor->SetMapper(mapper);

	return actor ;
}
