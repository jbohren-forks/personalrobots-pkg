/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


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
  : imWindow(x, y, w, h)
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
calImageWindow::clearAll()
{
  DeleteImage();
  num_pts2D = 0;
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
  if (num_pts2D > 0)
    drawBox(Im2WinX((int)(pts2D[0].x)), Im2WinY((int)(pts2D[0].y)), color);
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

