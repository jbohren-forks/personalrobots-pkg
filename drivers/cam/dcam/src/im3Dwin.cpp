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
 * im3Dwin.cpp
 *
 * OpenGL display in FLTK
 *
 *#########################################
 */

/**
 ** im3Dwin.cpp
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **/

#include <stdio.h>
#include <math.h>
#ifndef WIN32
#include <sys/time.h>
#endif

#include "imwin.h"
#include "im3Dwin.h"
#include "FL/Fl.H"


//
// GL graphics window definitions
//

im3DWindow::im3DWindow(int X, int Y, int W, int H, const char *L) 
  : Fl_Gl_Window(X, Y, W, H, L) 
{

  // effects: im3DWindow contstructor; creates the window and
  //          initializes the class variables

  anglex = 0;			// in degrees
  angley = 180;			// in degrees
  anglez = 0;			// in degrees
  newModel = 0;		    // should we recalculate a new model view?
  moving = 0;  // only needed for mouse interaction (disabled for now)
  numPoints = bufsize = 0; // number of 3D points we get from a disparity image
  axesLen = 1.0;		// display or don't display the axes

  scale = 1.0;			// don't scale for now
  scaleFactor = 4.0;
  xshift = 0.0;			// no shifts for now
  yshift = 0.0;
  zshift = 0.0;

  centerx = centery = 0;
  centerz = 1.0;

  gammaTable = new GLfloat[256];
  for (int i=0; i<256; i++)
    gammaTable[i] = (GLfloat)((double)i/255.0);
}
  
void im3DWindow::setDisplayAxes(double value)
{
  // requires: value = 0 or 1
  //  effects: sets the axes value to value.
  //           if value = 0, does not display axes on image.
  //           if value = 1, displays axes on image.

  axesLen = value;
  redraw();
}

void im3DWindow::scaleImage(float value)
{
  // requires: 0 <= value <= 1
  //  effects: scales the image according to an arbitrary
  //           constant (4) times value.

  scale = value * scaleFactor;
  newModel = 1;
  redraw();
}

void im3DWindow::transImageH(float value)
{
  // requires: 0 <= value <= 1
  //  effects: translates the image along the "horizontal"
  //           axis; a value of 0.5 is zero translation.

  xshift = (float)(0.5-value) * 8;
  newModel = 1;
  redraw();
}

void im3DWindow::transImageV(float value)
{
  // requires: 0 <= value <= 1
  //  effects: translates the image along the "vertical"
  //           axis; a value of 0.5 is zero translation.

  yshift = (float)(0.5-value) * 8;
  newModel = 1;
  redraw();
}

void im3DWindow::transImageZ(float value)
{
  // requires: 0 <= value <= 1
  //  effects: translates the image along the "z (depth)"
  //           axis; a value of 0.5 is zero translation.

  zshift = (float)(0.5-value) * 20;
  newModel = 1;
  redraw();
}

void im3DWindow::rotImageH(float value)
{
  // requires: 0 <= value <= 1
  //  effects: rotates the image around the "horizontal"
  //           axis; a value of 0.5 is zero rotation.

  anglex = (float)(0.5-value)*540;
  newModel = 1;
  redraw();

}

void im3DWindow::rotImageV(float value)
{
  // requires: 0 <= value <= 1
  //  effects: rotates the image around the "vertical"
  //           axis; a value of 0.5 is zero rotation.

  angley = (float)(0.5-value)*540 + 180;
  newModel = 1;
  redraw();
}

void im3DWindow::rotImageZ(float value)
{
  // requires: 0 <= value <= 1
  //  effects: rotates the image around the "z (depth)"
  //           axis; a value of 0.5 is zero rotation.

  anglez = (float)(0.5 - value)*540;
  newModel = 1;
  redraw();
}



// calcCenterZ
// effects: takes a disparity image and calculates average depth
// in a pixel neighborhood nh.
// assumes 3D has already been calculated

bool im3DWindow::calcCenterZ(StereoData *stIm, int nh)
{
#if 0
  int w, h;
  float *pts = stIm->imPts;
  float *pt;
  int np = 0;
  int ld, td;
  svsSP *sp = &di->sp;

  w = di->ip.width;   
  h = di->ip.height;
 
  // set center neighborhood boundaries
  ld = w/2 - nh/2;
  if (ld < 0)
    ld = 0;
  td = h/2 - nh/2;
  if (td < 0)
    td = 0;

  if (ld + nh > w)
    nh = w - ld - 1;
  if (td + nh > h)
    nh = h - td - 1;

  centerz = 0.0;
  float minz = 100000.0;
  int i, j;
  
  for (j=td; j<td+nh; j++)
    {
      pt = pts + j*w + ld;
      for (i=ld; i<ld+nh; i++, pt++)
	{
	  if (pt->A > 0.0) 
	    {
	      centerz = centerz + pt->Z;
              if (pt->Z < minz)
                minz = pt->Z;
	      np++;
	    }
	}
    }

  if (np > 0)
    {
      centerz = centerz/np;
      centerz = minz;		// try the minimum...
      if (centerz > 10)
	centerz = 10;
      return true;
    }
#endif

  return false;
}


// 
// Uses the 3D reconstruction and monochrome or color image
// to display a 3D rendering of the stereo image
//

void im3DWindow::DisplayImage(StereoData *stIm)
{
  int w, h;
  int ip = 0;
  float *pts;
  uint8_t *cc; 

  w = stIm->imWidth;
  h = stIm->imHeight;

  numPoints = stIm->numPts;
  if (numPoints > bufsize)	// check buffers
    {
      if (bufsize > 0)
	{
	  delete [] pointListX;
	  delete [] pointListY;
	  delete [] pointListZ;
	  delete [] colorListR;
	  delete [] colorListG;
	  delete [] colorListB;
	}
      bufsize = numPoints;
      pointListX = new GLfloat[bufsize];
      pointListY = new GLfloat[bufsize];
      pointListZ = new GLfloat[bufsize];
      colorListR = new GLfloat[bufsize];
      colorListG = new GLfloat[bufsize];
      colorListB = new GLfloat[bufsize];
    }
 
  cc = stIm->imPtsColor;
  pts = stIm->imPts;

  maxx = maxy = maxz = -1000;
  minx = miny = minz = 1000;

  if (stIm->isPtArray == false)	// just a vector 3xN of points
    {
      for (int i=0; i<numPoints; i++)
	{
	  pointListX[ip] = *pts++;
	  pointListY[ip] = *pts++;
	  pointListZ[ip] = *pts++;
	  colorListR[ip] = gammaTable[*cc++];
	  colorListG[ip] = gammaTable[*cc++];
	  colorListB[ip] = gammaTable[*cc++];
	  ip++;
	}
    }

  else				// array of points, corresponding to image
    {
#if 0
      int i, j;
      for (j=0; j<h; j++)
	{
	  cp =  j*w;
	  pts = di->pts3D + cp;	// 3D point array
	  for (i=0; i<w; i++, cp++, pts++)
	    {
	      if (pts->A > 0)	// invalid points have A = 0, -1, -2
		{
		  pointListX[ip] = pts->X;
		  pointListY[ip] = pts->Y;
		  pointListZ[ip] = pts->Z;
		  if (!cflag)	// monochrome
		    {
		      colorR = c[cp];
		      colorListR[ip] = colorListG[ip] = colorListB[ip] = gammaTable[colorR];
		    }
		  else 
		    {
		      colorR = cc[cp].r;
		      colorB = cc[cp].b;
		      colorG = cc[cp].g;
		      colorListR[ip] = gammaTable[colorR];
		      colorListG[ip] = gammaTable[colorG];
		      colorListB[ip] = gammaTable[colorB];
		    }
		  ip++;
		}
	    }
	}
#endif
    }

  numPoints = ip;

#if 0
  for (i=20; i<100; i+=10)
    {
      if (calcCenterZ(di,i))
	break;
      else
	centerz = 1.0;
    }
#endif

  //  printf("center: (%f, %f, %f)\n", centerx, centery, centerz);
  //  printf("%d/%d points, center (0,0,%d)\n", numPoints, di->numPoints, (int)centerz);
//  printf("extremes: ([%f, %f], [%f, %f], [%f, %f])\n", maxx, minx,
//	 maxy, miny, maxz, minz);

  newModel = 1;
  redraw();
}

void im3DWindow::recalcModelView() 
{
  // effects: recalculates the view of the model based on
  //          class variables (angle, scale, shift, center).
  //
  // rotates just around the centerpoint
  // translates from viewpoint
 
  glPopMatrix();
  glPushMatrix();

  glTranslatef(xshift, yshift, zshift);

  glRotatef(anglex, 0.0, 1.0, 0.0);
  glRotatef(angley, 1.0, 0.0, 0.0);
  glRotatef(anglez, 0.0, 0.0, 1.0);
  
  glScalef(scale, scale, scale);

//  glTranslatef(xshift*1000, yshift*1000, zshift*1000);

  glTranslatef(-centerx, -centery, -centerz);

  newModel = 0;  
}

void im3DWindow::draw() 
{
  // effects: draw function called when necessary to update
  //          the OpenGL window.
  //          draws the 3D points and axes.
  
  int i;

  if (!valid()) {
    // ... set up projection, viewport, etc ...
    // ... window size is in w() and h().
    // ... valid() is turned on by FLTK after draw() returns

    // hmm, should this be here?? or in an init somewhere?
	glLoadIdentity();
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glMatrixMode(GL_PROJECTION);
    gluPerspective(40.0,  // field of view in degree
		   1.0,   // aspect ratio  
		   0.5,   // Z near 
		   100.0);// Z far
    glMatrixMode(GL_MODELVIEW);
#if 0
    gluLookAt(0.0, 1.0, 8.0,  // eye location
	      0.0, 1.0, 0.0,  // center is at (0,0,0)
	      0.0, 1.0, 0.);  // up is in postivie Y direction 
#endif
    gluLookAt(0.5, 0.5, 4.0,  // eye location
	      0.5, 0.5, 0.0,  // what we look at
	      0.0, 1.0, 0.0); // up is in postive Y direction 
    glPushMatrix();    // dummy push so we can pop on model recalc
    
    glPointSize(1);
  }

  // ... draw ...

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(newModel)
    recalcModelView();

  glDisable(GL_TEXTURE_2D);


  glBegin(GL_POINTS);
  for(i=0; i<numPoints; i++) {
    glColor3f(colorListR[i], colorListG[i], colorListB[i]);
    glVertex3f(pointListX[i], pointListY[i], pointListZ[i]);
  }
  glEnd();

  if(axesLen) 
    {
      glBegin(GL_LINES);
      glColor3f(0, 0, 1);
      glVertex3f(0, 0, 0);
      glVertex3f(axesLen, 0, 0);
      glEnd();
    
      glBegin(GL_LINES);
      glColor3f(0, 1, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(0, axesLen, 0);
      glEnd();

      // this is the camera optical ray
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(0, 0, axesLen*20.0);
      glEnd();

#if 0
      // show centerpoint
      glBegin(GL_LINES);
      glColor3f(0, 0, 1);
      glVertex3f(centerx, centery, centerz);    
      glVertex3f(centerx, centery, centerz+1000);
      glEnd();
#endif
    }



}


int im3DWindow::handle(int event) 
{
  // effects: handles users events in the gl window
  // rotates the 3D model

  switch(event) {
    
  case FL_PUSH:
    // mouse down event
    moving = 1;
    beginx = Fl::event_x();
    beginy = Fl::event_y();
    return 1;
  case FL_DRAG:
    // mouse drag (while down) event
    if(moving) {
      anglex = anglex + (Fl::event_x()-beginx);
      beginx = Fl::event_x();
      angley = angley + (Fl::event_y()-beginy);
      beginy = Fl::event_y();
      newModel = 1;
      redraw();
    }
    return 1;
  case FL_RELEASE:
    // mouse up event
    moving = 0;
    return 1;
  default: 
    return Fl_Gl_Window::handle(event);
    
  }
}

