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
 * v3d.cpp
 *
 * simple 3D viewer based on OpenGL
 * can read ost-type PCD files
 *
 *#########################################
 */

/**
 ** v3d.cpp
 **
 ** Kurt Konolige
 ** Senior Researcher
 ** Willow Garage
 ** 68 Willow Road
 ** Menlo Park, CA 94025
 ** E-mail:  konolige@willowgarage.com
 **
 **/


#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <stdarg.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#ifdef WIN32
#pragma warning (disable: 4267 4244 4800 4996)
#include <time.h>
#else
#include <sys/time.h>
#endif
#include "imwin/im3Dwin.h"
#include "FL/Fl.H"

using namespace std;


// GUI stuff
im3DWindow *w3d;		// OpenGL 3D display
Fl_Window *w3dwin;		// Enclosing window
bool isExit;


// main program, just put up the GUI dialog

// int do_button(int e, int x, int y, int b, int m, imWindow *w);

int
main(int argc, char **argv)	// no arguments
{
  // parse args
  if (argc < 2)
    {
      printf("No file to load!  Give <file> as argument.\n");
      exit(0);
    }

  char *fname = argv[1];
  FILE *fd = fopen(fname,"rt");
  if (fd == NULL)
    {
      printf("Can't open file %s\n", fname);
      exit(0);
    }

  // read in file, create points to view
  vector<float> pts;		// points
  vector<float> cls;		// colors
  int tot=0;
  bool has_color = false;
  bool has_disp = false;
  while (1)
    {
      char line[1000];
      char *ll = line;		// ridiculous need to get types right for getline
      size_t nb = 999;
      ssize_t nbr = getline(&ll, &nb, fd);
      if (nbr < 0)
	break;			// done here
      float pt[6];		// for a point
      // parse tokens
      char *tk = strtok(ll," ");
      int n = 0;
      char *res;
      strtod(tk,&res);
      if (res == tk)
	continue;		// line without a number

      // parse a line
      for (int i=0; i<6; i++)
	{
	  if (tk)
	    {
	      n++;
	      float f = atof(tk);
	      pt[i] = f;
	    }
	  tk = strtok(NULL," ");
	}

      if (n == 4)
	has_disp = true;

      if (n == 6)
	has_color = true;

      // save pts
      if (!has_disp || pt[3] >= 0)
	{
	  pts.push_back(pt[0]);
	  pts.push_back(pt[1]);
	  pts.push_back(pt[2]);
	}
      else 
	continue;

      tot++;
      if (n == 6)		// save colors too
	{
	  cls.push_back(pt[3]);
	  cls.push_back(pt[4]);
	  cls.push_back(pt[5]);
	}

      if (n == 4)		// save monochrome colors too
	{
	  cls.push_back(pt[3]);
	  cls.push_back(pt[3]);
	  cls.push_back(pt[3]);
	}
    }

  printf("Found %d points\n",tot);

  w3d = NULL;			// no OpenGL window yet
  w3dwin = NULL;
  isExit = false;

  w3dwin = new Fl_Window(640,480,"3D Display");
  w3d = new im3DWindow(10,10,w3dwin->w()-20,w3dwin->h()-20);
  w3dwin->end();
  w3dwin->resizable(w3d);
  w3dwin->show();

  w3d->DisplayImage(pts,cls,0);

  while (Fl::check() && !isExit) // process GUI commands, serve video
    {}
}

//
// button handler
//

