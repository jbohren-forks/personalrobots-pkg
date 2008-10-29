/*#########################################
 * calwin.cpp
 *
 * display functions with feature drawing using FLTK window system
 *
 * (feature drawing should have been more general...
 *  this is very specific to the calibration routine)
 *
 *#########################################
 */

#include "calwin.h"

// 
// FLTK window definitions
//

// construct a double-buffered display window

calImageWindow::calImageWindow(int x, int y, int w, int h)
  : imWindow(x, y, w, h), 
  est2DPts(NULL), num_est2DPts(0)
{
  num_pts2D = 0;
  goodpts = false;
}


void 
calImageWindow::display2DFeatures(CvPoint2D32f *pts, int num_pts, bool good)
{
  pts2D = pts;
  num_pts2D = num_pts;
  goodpts = good;
  redraw();
}


void 
calImageWindow::clear2DFeatures()
{
  num_pts2D = 0;
  redraw();
}

void 
calImageWindow::displayEst2DFeatures(CvPoint2D32f *pts, int num_pts)
{
  est2DPts = pts;
  num_est2DPts = num_pts;
  redraw();
}


void 
calImageWindow::clearEst2DFeatures()
{
  num_est2DPts = 0;
  redraw();
}



void 
calImageWindow::clearAll()
{
  DeleteImage();
  num_pts2D = 0;
  num_est2DPts = 0;
  imWindow::redraw();
  redraw();
}


// replace this with overlay
void
calImageWindow::draw() 
{
  int i;
  imWindow::draw();

  Fl_Color color = FL_GREEN;
  if (!goodpts)
    color = FL_RED;

  for(i=0; i<num_pts2D; i++)
	    drawCross(Im2WinX((int)(pts2D[i].x)), Im2WinY((int)(pts2D[i].y)), color);
  
  // draw a box over the first point
  if (num_pts2D > 0)
    drawBox(Im2WinX((int)(pts2D[0].x)), Im2WinY((int)(pts2D[0].y)), color);
  
  // if any, draw estimated points from estimated transformation
  Fl_Color estPtsColor = FL_YELLOW;
  for (i=0; i<num_est2DPts; i++) {
//	    drawCross(Im2WinX((int)round(est2DPts[i].x)), Im2WinY((int)round(est2DPts[i].y)), estPtsColor);
	    drawCross(Im2WinX((int)(est2DPts[i].x)), Im2WinY((int)(est2DPts[i].y)), estPtsColor);
  }
  if (num_est2DPts > 0)
    drawBox(Im2WinX((int)(est2DPts[0].x)), Im2WinY((int)(est2DPts[0].y)), estPtsColor);
}			

void
calImageWindow::drawCross(int x, int y, Fl_Color color)
{
  fl_color(color);
  fl_line(x, y-2, x, y+2);
  fl_line(x-2, y, x+2, y);
}

void
calImageWindow::drawBox(int x, int y, Fl_Color color)
{
  fl_color(color);
  fl_rect(x-2,y-2,5,5);
}

