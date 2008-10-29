// 
// FLTK feature window definitions
//

#ifndef calwin_h
#define calwin_h

#include "imwin.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class calImageWindow : public imWindow
{
public:
  calImageWindow(int x, int y, int h, int w);

  void display2DFeatures(CvPoint2D32f *pts, int num_pts, bool good);
  void clear2DFeatures();
  
  void displayEst2DFeatures(CvPoint2D32f *pts, int num_pts);
  void clearEst2DFeatures();

  void clearAll();

  void draw();     // drawing routine

 private: 
  void drawCross(int x, int y, Fl_Color color);
  void drawBox(int x, int y, Fl_Color color);

  CvPoint2D32f *pts2D;
  int num_pts2D;
  bool goodpts;
  
  CvPoint2D32f *est2DPts; // points tranform from another position by estimated transformation
  int num_est2DPts;

};

#endif
