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
 * imwin.cpp
 *
 * display functions using FLTK window system
 *
 *#########################################
 */

/**
 ** imwin.cpp
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
#include <stdarg.h>
#include <math.h>
#define LIBCODE
#include "imwin.h"

// 
// FLTK window definitions
//

// colormap for disparities
// colormap
static unsigned char dmap[768] = 
  { 150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    0, 0, 0,
    0, 0, 0
  };



// construct a display window

imWindow::imWindow(int x, int y, int w, int h, char *name)
  : Fl_Window(x,y,w,h,name)
{
  width = dwidth = w;
  height = dheight = h;
  xoff = yoff = 0;
  skipw = 1;
  linesize = w;
  pixelData = NULL;
  bufSize = curSize = 0;
  pixelType = MONOCHROME;
  gamma = 0.0;			// no gamma
  //  end();
  bhandler = NULL;
  khandler = NULL;
  ofn      = NULL;
  myColor  = FL_BLACK;
  intX = intY = intW = intH = 0;
  moving = false;
  ovArg = NULL;
  saveName = "image";
  saveNum = 0;
  saveBuf = NULL;
}

imWindow::imWindow(int w, int h, char *name)
  : Fl_Window(w,h,name)
{
  width = dwidth = w;
  height = dheight = h;
  xoff = yoff = 0;
  skipw = 1;
  linesize = w;
  pixelData = NULL;
  bufSize = curSize = 0;
  pixelType = MONOCHROME;
  gamma = 0.0;			// no gamma
  //  end();
  bhandler = NULL;
  khandler = NULL;
  ofn      = NULL;
  myColor  = FL_BLACK;
  intX = intY = intW = intH = 0;
  moving = false;
  ovArg = NULL;
  saveName = "image";
  saveNum = 0;
  saveBuf = NULL;
}

IMPORT void
imWindow::ClearImage()
{
  if (pixelData)
    memset(pixelData, 0, curSize);
  redraw();
}

IMPORT void
imWindow::DeleteImage()
{
  if (pixelData)
    delete [] pixelData;
  pixelData = NULL;
  bufSize = 0;
  redraw();
}



// Scale from image to window

IMPORT int
imWindow::Win2ImX(int x)
{
  return x*skipw;
}

IMPORT int
imWindow::Win2ImY(int x)
{
  return x*skipw;
}

int
imWindow::Im2WinX(int x)
{
  return x/skipw;
}

int
imWindow::Im2WinY(int x)
{
  return x/skipw;
}


//
// image display function
// will try to decimate image to fit in display window
// disparity images are 16 bits/pixel, with ndisp values
//

void
imWindow::DisplayImage(unsigned char *im, int w, int h, int ls, int type, int nd, 
                        int s, double gam, int x, int y)
{
  // save these for redraws
  int size = w*h;
  pixelType = type;
  if (pixelType == RGB24)
    size = size * 3;
  if (pixelType == DISPARITY)
    size = size * 2;

  // check the buffer
  if (bufSize < size)		
    {
      if (pixelData)
        delete [] pixelData;
      pixelData = new unsigned char[size];
      bufSize = size;
    }
  curSize = size;

  memcpy(pixelData, im, curSize);

  xoff = x;
  yoff = y;
  width = w;
  height = h;
  linesize = ls;
  skipw = s;
  ndisp = nd;
  if (gam != gamma)
    {
      gamma = gam;
      if (gam > 0.0 && gam < 1.0)
	{
	  for (int i=0; i<256; i++)
	    gamtab[i] = (int)(0.5 + 255.0 * pow((double)i/255.0, gam));
	}
    }


  // image scaling, keeps aspect ratio
  while (height > dheight) 
    {
      height = height / 2;
      yoff = yoff / 2;
      linesize *= 2;
      width = width / 2;
      skipw *= 2;
      xoff = xoff / 2;
    }

  while (width > dwidth)
    {
      width = width / 2;
      skipw *= 2;
      xoff = xoff / 2;
      height = height / 2;
      yoff = yoff / 2;
      linesize *= 2;
    }

  redraw();
}


struct image_cb_data
{
  unsigned char *pdata;
  int ndisp;
  int skip;
  int linesize;
  unsigned char *gtab;		// table of gamma values
};

// copy n pixels to out, starting at x,y
// changed to draw color, based on colormap
static void
disp_image_cb_fn(void *data, int x, int y, int n, uchar *out)
{
  int i, v, ndisp, skip;
  short *in;
  struct image_cb_data *cbd = (struct image_cb_data *)data;

  in = (short *)cbd->pdata;	// disparity images are short's
  in += y*cbd->linesize + x;
  ndisp = cbd->ndisp;
  skip = cbd->skip;
  for (i=0; i<n; i++, in+=skip)
    {
      v = *in;
      if (v < 0)
	{
	  *out++ = 0;
	  *out++ = 0;
	  *out++ = 0;
	}      
      else
	{
	  v = (v*255)/ndisp;
	  v = v*3;
	  *out++ = dmap[v];
	  *out++ = dmap[v+1];
	  *out++ = dmap[v+2];
	}
    }
};

// copy n pixels to out, starting at x,y
static void
gamma_image_cb_fn(void *data, int x, int y, int n, uchar *out)
{
  int i, ndisp, skip;
  unsigned char *in;
  unsigned char *gtab;
  struct image_cb_data *cbd = (struct image_cb_data *)data;
  in = (unsigned char *)cbd->pdata;	// mono image
  in += y*cbd->linesize + x;
  ndisp = cbd->ndisp;
  skip = cbd->skip;
  gtab = cbd->gtab;
  for (i=0; i<n; i++, in+=skip)
    *out++ = gtab[*in];
};


// copy n pixels to out, starting at x,y
static void
gamma_color_image_cb_fn(void *data, int x, int y, int n, uchar *out)
{
  int i, ndisp, skip;
  unsigned char *in;
  unsigned char *gtab;
  struct image_cb_data *cbd = (struct image_cb_data *)data;
  in = (unsigned char *)cbd->pdata;	// RGB24 image
  in += y*cbd->linesize + x;
  ndisp = cbd->ndisp;
  skip = cbd->skip;
  gtab = cbd->gtab;
  for (i=0; i<n; i++, in+=skip)
    {
      *out++ = gtab[*in];
      *out++ = gtab[*(in+1)];
      *out++ = gtab[*(in+2)];
    }
};


void
imWindow::drawit()
{
  if (!shown()) return;

  struct image_cb_data cbd;
  //  make_current();

  if (pixelType == MONOCHROME)
    {
      if (gamma > 0.0 && gamma < 1.0)
	{
	  cbd.pdata = pixelData;
	  cbd.ndisp = ndisp;
	  cbd.linesize = linesize;
	  cbd.skip = skipw;
	  cbd.gtab = gamtab;
	  fl_draw_image_mono(gamma_image_cb_fn, (void *)&cbd, xoff, yoff, width, height, 1);
	}
      else
        fl_draw_image_mono(pixelData, xoff, yoff, width, height, skipw, linesize);
    }
  else if (pixelType == RGB24)
    {
      if (gamma > 0.0 && gamma < 1.0)
	{
	  cbd.pdata = pixelData;
	  cbd.ndisp = ndisp;
	  cbd.linesize = linesize*3;
	  cbd.skip = skipw*3;
	  cbd.gtab = gamtab;
	  fl_draw_image(gamma_color_image_cb_fn, (void *)&cbd, xoff, yoff, width, height, 3);
	}
      else
	fl_draw_image(pixelData, xoff, yoff, width, height, skipw*3, linesize*3);
    }

  else if (pixelType == DISPARITY)
    {
      cbd.pdata = pixelData;
      cbd.ndisp = ndisp;
      cbd.linesize = linesize;
      cbd.skip = skipw;
      fl_draw_image(disp_image_cb_fn, (void *)&cbd, xoff, yoff, width, height, 3);
    }
}



// overlay drawing

void				// return the image pixel coords of the interest region
imWindow::imageRegion(int *x, int *y, int *w, int *h)
{
  *x = intX * skipw;
  *y = intY * skipw;
  *w = intW * skipw;
  *h = intH * skipw;
}

void
imWindow::DrawOverlay(int which, void *arg)
{
  // first, draw an interest rectangle
  if (intW > 0 && intH > 0)
    {
      fl_color(FL_RED);
      fl_line(intX, intY, intX+intW, intY);
      fl_line(intX, intY, intX, intY+intH);
      fl_line(intX+intW, intY, intX+intW, intY+intH);
      fl_line(intX, intY+intH, intX+intW, intY+intH);
    }

  // now check for any other fn
  if (ofn)
    (* ofn)(this, which, arg);
}


void
imWindow::line(int x1, int y1, int x2, int y2)
{
  fl_color(myColor);
  fl_line(x1,y1,x2,y2);
}

void
imWindow::rect(int x, int y, int w, int h)
{
  fl_color(myColor);
  fl_line(x,y,x+w,y);
  fl_line(x,y,x,y+h);
  fl_line(x+w,y,x+w,y+w);
  fl_line(x,y+w,x+w,y+w);
}



// re-drawing function, copies pixelData to the image
void
imWindow::draw()
{
  if (height < dheight || width < dwidth || !pixelData)
    {
      fl_color(FL_BLACK);
      fl_rectf(0,0,dwidth,dheight);
    }
  if (pixelData)
    drawit();
  DrawOverlay(myWhich, ovArg);
}

// saving to a file
void
imWindow::Save(char *fname, int inum)
{
  if (saveBuf == NULL)
    saveBuf = new unsigned char[dwidth*dheight*4];
  make_current();
  // link error in MSW????
  //  fl_read_image(saveBuf, 0, 0, dwidth, dheight); // get image into buffer 

  if (inum >= 0)
    saveNum = inum;

  if (fname != NULL)
    saveName = fname;

  char name[256];
  sprintf(name, "%s-%05d.bmp", saveName, saveNum);
  saveNum++;

  //  if (fname != NULL && inum < 0) // just use fname
  //    imWriteImageColorBMP(fname, saveBuf, dwidth, dheight, 0);
  //  else
  //    imWriteImageColorBMP(name, saveBuf, dwidth, dheight, 0);
}			


// mouse event handling, let the user specify a callback via AddButtonHandler

int 
imWindow::handle(int e) 
{
  int key, mod, ret;
  int x, y;
  if (Fl_Window::handle(e)) return 1;
  mod = Fl::event_state();	// modifiers (shift, alt, etc)
  key = Fl::event_key();
  int b = (mod & 0x0F000000);
  x = Fl::event_x();
  y = Fl::event_y();

  if (bhandler && (e == FL_PUSH || e == FL_DRAG || e == FL_RELEASE))
    {
      if (Fl::focus() != this)
        Fl::focus(this);
      ret = bhandler(e, x, y, b, (mod & 0x00FF0000), this);
      if (ret != 0)
        return ret;
    }

  // right button, set up interest rectangle
  if (b == FL_BUTTON3 && (e == FL_PUSH || e == FL_DRAG || e == FL_RELEASE))	
    {
      if (e == FL_PUSH)
        {
          moving = true;
          intX = x;
          intY = y;
          intW = intH = 0;
          redraw();
        }

      if (e == FL_RELEASE || e == FL_DRAG)
        {
          if (e == FL_RELEASE) moving = false;
          if (x != intX && y != intY) // ok, have a rectangle
            {
              if (x < intX) { int xx = x; x = intX; intX = xx; }
              if (y < intY) { int yy = y; y = intY; intY = yy; }
              intW = x - intX;
              intH = y - intY;
            }
          else
            intW = intH = 0;
          redraw();
        }

      return 1;
    }


  if (e == FL_KEYBOARD && key < 256 && khandler)
    {
      ret = khandler(key, (mod >> 16) & 0xF, this);
      return ret;
    }

  return 0;
}




//
// value widget
//

// Fl_Value creation

Fl_Value::Fl_Value(int x, int y, int w, int h, const char *l, const char *val)
  : Fl_Widget(x,y,w,h,l)
{ 
  strcpy(str,val); 
  strcpy(buf, val); 
  align(FL_ALIGN_LEFT); 
  labelcolor(FL_BLACK); 
  labelfont(FL_HELVETICA);	// we need this if the size is to change
  labelsize(14);
  c = FL_BLACK;
}

void
Fl_Value::val(char *v)
{ 
  if (v) 
    { 
      strcpy(str,v); 
      strcpy(buf,v); 
    }
  redraw(); 
}

void
Fl_Value::val(int x)
{ 
  sprintf(buf, str, x); 
  redraw(); 
}

void
Fl_Value::val(double x)
{ 
  sprintf(buf, str, x); 
  redraw(); 
}


//
// debug window
//

void copy_cb(Fl_Widget*, void* v) {
  imInfoWindow* e = (imInfoWindow*)v;

  if (!e->dbuf->selected()) return;
  const char *copy = e->dbuf->selection_text();
  if (*copy) Fl::copy(copy, strlen(copy), 1);
}

void save_cb(Fl_Widget*, void* v) {
  imInfoWindow* e = (imInfoWindow*)v;
  char *newfile;
  newfile = fl_file_chooser("Save File As?", "*.txt", "");
  if (newfile != NULL) 
    e->Save(newfile);
}


bool
imInfoWindow::Save(char *fname)
{
  bool ret = false;
  if (dbuf != NULL && fname != NULL)
    {
      if (dbuf->savefile(fname) == 0)
	ret = true;
    }
  return ret;
}


Fl_Menu_Item menuitems[] = {
  { "&File",              0, 0, 0, FL_SUBMENU },
  { "&Save File",       FL_CTRL + 's', (Fl_Callback *)save_cb },
  //    { "E&xit", FL_CTRL + 'q', (Fl_Callback *)quit_cb, 0 },
    { 0 },

  { "&Edit", 0, 0, 0, FL_SUBMENU },
  { "&Copy",       FL_CTRL + 'c', (Fl_Callback *)copy_cb },
  { 0 },

  { 0 }
};


imInfoWindow::imInfoWindow(int w, int h, char *name)
  : Fl_Window(w,h,name)
{
  dwin = new Fl_Text_Display(0,30,w,h);
  dbuf = new Fl_Text_Buffer;
  dwin->buffer(dbuf);
  resizable(dwin);
  Fl_Menu_Bar* m = new Fl_Menu_Bar(0, 0, w, 30);
  m->copy(menuitems, this);
  end();

  dbuf->append("OST info window\n");
  dbuf->append("\n");
}

void
imInfoWindow::Print(char *str, ...)
{
  va_list ptr;
  va_start(ptr,str);
  if (str && dbuf)
    {
      vsprintf(outp, str, ptr);
      dbuf->append(outp);
      dbuf->append("\n");
      Bottom();
    }
  va_end(ptr);
}

void
imInfoWindow::Bottom()
{
  dwin->insert_position(dbuf->length());
  dwin->show_insert_position();
}




// drawing fns

int fltk_check(void)
  {
    return Fl::check();
  }
