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

// 
// FLTK window definitions
//

#ifndef flwin_h
#define flwin_h

#ifdef WIN32
#pragma warning (disable: 4311 4312 4267 4996)
#endif

#define index xx		// do we need this????  FLTK sometimes has problems
#undef index
#include "FL/Fl.H"
#include "FL/Fl_Group.H"
#include "FL/Fl_Window.H"
#include "FL/fl_draw.H"
#include "FL/Fl_Double_Window.H"
#include "FL/Fl_Browser.H"
#include "FL/Fl_Overlay_Window.H"
#include "FL/Fl_Select_Browser.H"
#include "FL/Fl_Text_Display.H"
#include "FL/Fl_Text_Buffer.H"
#include "FL/Fl_Menu_Bar.H"
#include "FL/Fl_File_Chooser.H"
#include <string.h>


// for MSVC this should be defined correctly
#define IMPORT 

// pixel types
#define MONOCHROME 0
#define RGB24      1
#define DISPARITY  2

class imWindow : public Fl_Double_Window
{
public:
  IMPORT imWindow(int x, int y, int w, int h, char *name = "Image Window");
  IMPORT imWindow(int w, int h, char *name = "Image Window");
  IMPORT void ClearImage();
  IMPORT void DeleteImage();
  IMPORT void draw();           // drawing routine 
  IMPORT void DisplayImage(unsigned char *im, int w, int h, int ls, int type = MONOCHROME, 
                           int nd = 64, int s = 1, double gam = 0.0, int x = 0, int y = 0);
  // draw overlays
  virtual void DrawOverlay(int which, void *); // draw overlays with overlay fn; can be overridden

  // if we want to do something with mouse clicks...
  void ButtonHandler(int (*fn)(int, int, int, int, int, imWindow *))
  { bhandler = fn; };
  // if we want to do something with keyboard presses...
  void KeyHandler(int (*fn)(int, int, imWindow *))
  { khandler = fn; };
  // set up overlay function
  void DrawOverlayFn(void (*fn)(imWindow *, int which, void *ovArg))
  { ofn = fn; };

  IMPORT void line(int x1, int y1, int x2, int y2); // draw line on overlay
  IMPORT void rect(int x1, int y1, int x2, int y2); // draw rectangle on overlay
  void color(int c)             // set the color
  { myColor = c; };

  // return the image pixel coords of the interest region
  void imageRegion(int *x, int *y, int *w, int *h);
  int intX, intY, intW, intH; // interest rectangle

  int dwidth, dheight;          // display window size, doesn't change

  // scale image coords to window coords
  IMPORT int Win2ImX(int x);
  IMPORT int Win2ImY(int y);
  IMPORT int Im2WinX(int x);
  IMPORT int Im2WinY(int y);

  // saving to a file
  // saves to a file sequence, if no arguments are given
  IMPORT void Save(char *fname = NULL, int num = -1);
  int saveNum;
  char *saveName;		// base file name

  void *data;

private: 
  void drawit();                // draws the pixmap on the screen
  int width, height;            // displayed image size
  int xoff, yoff;               // displayed image offset withing display window
  int skipw, linesize;          // pixel decimation, line size of original image 
  unsigned char *pixelData;
  int pixelType;
  int curSize, bufSize;         // internal buffering
  int ndisp;
  double gamma;			// for gamma conversion
  unsigned char gamtab[256];
  // to check key and mouse events
  IMPORT int handle(int);       // has to be IMPORTed so subclasses can see it
  bool moving;                  // dragging the mouse
  int (*bhandler)(int, int, int, int, int, imWindow *);
  int (*khandler)(int, int, imWindow *);
  void (* ofn)(imWindow *, int which, void *ovarg); // set this to be the overlay drawing fn
  int myColor;
  void *ovArg;                  // stored arg to overlay fn
  int myWhich;                  // stored which image for overlay fn
  unsigned char *saveBuf;	// buffer for saved images
};


//
// Draws a single string, int, or double value, with an associated label
// Can change formatting and type at any time; value() fn is overloaded
// value(char *) acts as formatter for numeric values
// If formatter is changed, the numeric value must be resent
//

class Fl_Value : public Fl_Widget 
{

public:
  Fl_Value(int x, int y, int w, int h, const char *l=0, const char *val = "");
  void val(char *v); 
  void val(int x);
  void val(double x);
  void setcolor(Fl_Color x) { c = x; }

private:
  void draw()
  { 
    fl_color(color());
    fl_rectf(x(),y(),w(),h());  // erase background
    fl_font(FL_HELVETICA_BOLD, labelsize());
    fl_color(c);
    fl_draw(buf, x(), y()+ h()/2 + fl_height()/2 - fl_descent() );
  }
  Fl_Color c;
  char str[256];
  char buf[256];
};


class imInfoWindow : public Fl_Window
{
public:
  IMPORT imInfoWindow(int, int, char *name = NULL);
  IMPORT void Print(char *str, ...);
  IMPORT void Bottom();
  IMPORT bool Save(char *filename); // saves to a named file
  Fl_Text_Display *dwin;
  Fl_Text_Buffer *dbuf;
private:
  char outp[1024];
};

IMPORT int fltk_check(void);

#endif

