//
// simple timing test of the nav fn planner
// 

#include "navwin.h"
#include <string.h>


NavWin::NavWin(int w, int h, char *name)
  : Fl_Double_Window(w,h,name)
{
  nw = w;
  nh = h;
  dec = 1;
  maxval = 90*1000;
  im = NULL;
  pw = w;
  ph = h;
  pce = pne = poe = 0;
  pc = pn = po = NULL;
}

NavWin::~NavWin()
{
}

void
NavWin::drawPot(NavFn *nav)
{
  float *pot = nav->potarr;
  int width = nav->nx;
  int height = nav->ny;

  // params
  pw = width;
  ph = height;

  // figure out decimation or expansion to fit
  dec = 1;
  inc = 1;

  if (width >= nw/2)
    {
      int ww = width; 
      while (ww > nw)
	{
	  dec++;
	  ww = width/dec;
	}

      int hh = height/dec;
      while (hh > nh)
	{
	  dec++;
	  hh = height/dec;
	}

      if (im == NULL)
	im = new uchar[nw*nh];

      for (int i=0; i<height-dec+1; i+=dec)
	{
	  float *pp = pot + i*width;
	  uchar *ii = im + i/dec * nw;
	  for (int j=0; j<width-dec+1; j+=dec, pp+=dec)
	    {
	      int v;
	      if (*pp > maxval)
		v = 255;
	      else
		v = (int)((*pp/maxval) * 255.0);
	      *ii++ = v;
	    }
	}
    }

  else				// expand
    {
      int ww = width; 
      while (ww < nw/2)
	{
	  inc++;
	  ww = width*inc;
	}

      int hh = height*inc;
      while (hh < nh/2)
	{
	  inc++;
	  hh = height*inc;
	}

      if (im == NULL)
	im = new uchar[nw*nh];

      for (int i=0; i<height; i++)
	{
	  float *pp = pot + i*width;
	  uchar *ii = im + i*inc * nw;
	  for (int j=0; j<width; j++, pp++)
	    {
	      int v;
	      if (*pp > maxval)
		v = 255;
	      else
		v = (int)((*pp/maxval) * 255.0);
	      for (int k=0; k<inc; k++)
		{
		  uchar *iii = ii + j*inc + k*nw;
		  for (int kk=0; kk<inc; kk++)
		    *iii++ = v;
		}
	    }
	}
    }


  make_current();
  fl_draw_image_mono(im, 0,0,nw,nh);

  if (pc)
    delete [] pc;
  pce = nav->curPe;
  pc = new int[pce];
  memcpy(pc, nav->curP, pce*sizeof(int));

  if (pn)
    delete [] pn;
  pne = nav->nextPe;
  pn = new int[pne];
  memcpy(pn, nav->nextP, pne*sizeof(int));

  if (po)
    delete [] po;
  poe = nav->overPe;
  po = new int[poe];
  memcpy(po, nav->overP, poe*sizeof(int));

  goal[0] = nav->goal[0];
  goal[1] = nav->goal[1];

  drawOverlay();

  redraw();
}


void 
NavWin::drawOverlay()
{
  make_current();

  if (inc == 1)			// decimation
    {
      fl_color(255,0,0);
      if (pce > 0)
	for (int i=0; i<pce; i++)
	  {
	    int y = pc[i]/pw;
	    int x = pc[i]%pw;
	    fl_point(x/dec,y/dec);
	  }
      fl_color(255,0,0);
      if (pne > 0)
	for (int i=0; i<pne; i++)
	  {
	    int y = pn[i]/pw;
	    int x = pn[i]%pw;
	    fl_point(x/dec,y/dec);
	  }
      fl_color(0,255,0);
      if (poe > 0)
	for (int i=0; i<poe; i++)
	  {
	    int y = po[i]/pw;
	    int x = po[i]%pw;
	    fl_point(x/dec,y/dec);
	  }
    }
  else				// expansion
    {
      fl_color(255,0,0);
      if (pce > 0)
	for (int i=0; i<pce; i++)
	  {
	    int y = pc[i]/pw;
	    int x = pc[i]%pw;
	    fl_rect(x*inc,y*inc,inc,inc);
	  }
      fl_color(255,0,0);
      if (pne > 0)
	for (int i=0; i<pne; i++)
	  {
	    int y = pn[i]/pw;
	    int x = pn[i]%pw;
	    fl_rect(x*inc,y*inc,inc,inc);
	  }
      fl_color(0,255,0);
      if (poe > 0)
	for (int i=0; i<poe; i++)
	  {
	    int y = po[i]/pw;
	    int x = po[i]%pw;
	    fl_rect(x*inc,y*inc,inc,inc);
	  }
    }

  // draw the goal
  fl_color(255,200,0);
  int x = goal[0];
  int y = goal[1];
  if (inc == 1)
    fl_rectf(x/dec-2,y/dec-2,5,5);      
  else
    fl_rectf(x*inc-2,y*inc-2,5,5);      


}

void NavWin::draw()
{
  if (im)
    fl_draw_image_mono(im, 0,0,nw,nh);
  drawOverlay();
}

