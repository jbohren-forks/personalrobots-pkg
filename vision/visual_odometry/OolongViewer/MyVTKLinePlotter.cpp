#include "MyVTKLinePlotter.h"



#include "vtkLookupTable.h" ;
#include "vtkPoints.h"
#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkPointData.h"
#include "vtkProperty.h"


MyVTKLinePlotter::MyVTKLinePlotter()
:m_scalarMin(0.0), m_scalarMax(1.0), 
m_lookupTable(NULL), m_curPointID(0), m_allLineWidth(1)
{
	m_points = vtkPoints::New();
	m_lines = vtkCellArray::New();
	m_lineScalars = vtkFloatArray::New();
}

void MyVTKLinePlotter::SetScalarRange(double minval, double maxval)
{
	m_scalarMin = minval ;
	m_scalarMax = maxval ;
}
void MyVTKLinePlotter::SetLookupTable(vtkLookupTable* table)
{
	m_lookupTable = table ;
}
void MyVTKLinePlotter::PlotLine(double m[3], double n[3], double scalar)
{

	m_lineScalars->SetNumberOfComponents(1);
	m_points->InsertNextPoint(m);
	m_lineScalars->InsertNextTuple1(scalar);
	m_points->InsertNextPoint(n);
	m_lineScalars->InsertNextTuple1(scalar);

	m_lines->InsertNextCell(2);
	m_lines->InsertCellPoint(m_curPointID);
	m_lines->InsertCellPoint(m_curPointID+1);

	m_curPointID+=2;
}
void MyVTKLinePlotter::PlotLine(double x, double y, double z,
		double x2, double y2, double z2, double scalar)
{
	double m[3],n[3] ;
	m[0]=x; m[1]=y; m[2]=z;
	n[0]=x2; n[1]=y2; n[2]=z2;
	PlotLine(m,n,scalar);
	
}


void MyVTKLinePlotter::SetAllLineWidth(int width)
{
	m_allLineWidth = width ;
}

vtkPolyData* MyVTKLinePlotter::CreatePolyData()
{
	// Create poly data 
	vtkPolyData* polyData = vtkPolyData::New();
	polyData->SetPoints(m_points);
	polyData->SetLines(m_lines);
	polyData->GetPointData()->SetScalars(m_lineScalars);

	return polyData;
}
vtkActor* MyVTKLinePlotter::CreateActor()
{
	
	// Create poly data
	vtkPolyData* polyData = CreatePolyData();

	// create a color lookup table
	if (m_lookupTable==NULL)	
	{
		m_lookupTable = vtkLookupTable::New();
	}
		
	// create mapper
	vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
	mapper->SetInput(polyData);
	mapper->SetLookupTable(m_lookupTable);

	mapper->SetColorModeToMapScalars();
	mapper->SetScalarRange(m_scalarMin, m_scalarMax);
	mapper->SetScalarModeToUsePointData();

	// create actor
	vtkActor* actor = vtkActor::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetLineWidth(m_allLineWidth);


	return actor ;	
}
