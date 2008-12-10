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
 * im3Dwin.h
 *
 * OpenGL display in FLTK
 *
 *#########################################
 */

/**
 ** im3Dwin.h
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **/


#ifndef _IM3DWIN_H_
#define _IM3DWIN_H_

#include "FL/Fl_Gl_Window.H"
#include "FL/gl.h"
#include <GL/glut.h>
#include <GL/glu.h>
#include "image.h"

using namespace cam;

//
// Class for drawing 3D information using OpenGL
//

class im3DWindow : public Fl_Gl_Window {
  
 public:
  
  im3DWindow(int X, int Y, int W, int H); // constructor

  // set up 3D image from disparity
  void DisplayImage(StereoData *stIm);
  // set up 3D image from points
  void DisplayImage(float *imPts, int type);

  // orientation and scaling
  void scaleImage(float value);	// scales the image with a fixed aspect ratio
  void transImageH(float value); // translate the 3D image horizontally
  void transImageV(float value); // translate the 3D image vertically
  void transImageZ(float value); // translate the 3D image depth-wise
  void rotImageH(float value);	// rotate the 3D image horizontally
  void rotImageV(float value);	// rotate the 3D image vertically
  void rotImageZ(float value);	// rotate the 3D image depth-wise

  void setDisplayAxes(double value); // display or don't display the axes (value = 1 or 0)

 private:

  int xs, ys;		    // window size
  void draw();              // draw function called when necessary by FLTK
  int handle(int);          // user event handling function, called by FLTK
  void resize(int X, int Y, int W, int H);

  GLfloat *colorListR, *colorListG, *colorListB; // point colors
  GLfloat *pointListX, *pointListY, *pointListZ; // points 
  GLfloat *gammaTable;

  void recalcModelView();   // recalculate 3D view
  // this finds average Z over a center pixel neighborhood
  bool calcCenterZ(StereoData *stIm, int nh);

  int moving, beginx, beginy, numPoints, numVecs, newModel;
  double axesLen;

  GLfloat anglex, angley, anglez;      // 3D view paramaters
  GLfloat scale, scaleFactor;
  GLfloat xshift,  yshift, zshift;
  
  float centerx, centery, centerz;     // parameters to make viewing more intuitive
  float minx, miny, minz; 
  float maxx, maxy, maxz;
  float pointScale;		// scale points to mm

  int bufsize, vbufsize;

};

#endif  // _IM3DWIN_H_

