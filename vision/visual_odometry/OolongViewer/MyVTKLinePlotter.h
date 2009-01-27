/////////////////////////////////////////////////////////////////////////////////////
// Name:        MyVTKLinePlotter.h
// Purpose:     Plot a lot of straight line, and create a single actor to render
// Author:		Chung Kai Lun Peter
////////////////////////////////////////////////////////////////////////////////////

#ifndef MY_VTK_LINE_PLOTTER_H
#define MY_VTK_LINE_PLOTTER_H

class vtkLookupTable ;
class vtkPoints ;
class vtkCellArray ;
class vtkFloatArray ;
class vtkActor ;
class vtkPolyData;


class MyVTKLinePlotter
{
public:

	MyVTKLinePlotter();

	void SetScalarRange(double minval=0.0, double maxval=1.0);
	void SetLookupTable(vtkLookupTable* table = 0);

	void SetAllLineWidth(int width = 1);
	void PlotLine(double m[3], double n[3], double scalar);
	void PlotLine(double x, double y, double z,
		double x2, double y2, double z2, double scalar);
	
	vtkActor* CreateActor(); // call after all lines are plotted!
	vtkPolyData* CreatePolyData(); // call after all lines are plotted!
private:

	double m_scalarMin, m_scalarMax ;
	vtkLookupTable* m_lookupTable ;
	int m_curPointID ;
	int m_allLineWidth ;

	vtkPoints* m_points;
	vtkCellArray* m_lines;
	vtkFloatArray* m_lineScalars ;
	
};

#endif // MY_VTK_LINE_PLOTTER_H