//
// simple timing test of the nav fn planner
// 

#include "navfn.h"
#include "navwin.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern "C" {
#include <stdio.h>
// pgm.h is not very friendly with system headers... need to undef max() and min() afterwards
#include <pgm.h>
#undef max
#undef min
}
#define unknown_gray 0xCC	// seems to be the value of "unknown" in maps


int goal[2];
int start[2];

double get_ms()
{
  struct timeval t0;
  gettimeofday(&t0,NULL);
  double ret = t0.tv_sec * 1000.0;
  ret += ((double)t0.tv_usec)*0.001;
  return ret;
}

NavWin *nwin;

void
dispPot(NavFn *nav)
{
  nwin->drawPot(nav);
  Fl::check();
}

COSTTYPE *readPGM(char *fname, int *width, int *height);

int main(int argc, char **argv)
{
  int dispn = 0;

  int res = 50;			// 50 mm resolution
  double size = 40.0;		// 40 m on a side
  int inc = COST_OBS+10;	// thin wavefront
  
  // get resolution (mm) and perhaps size (m)
  if (argc > 1)
    res = atoi(argv[1]);

  if (argc > 2)
    size = atoi(argv[2]);

  if (argc > 3)
    inc = atoi(argv[3]);

  if (argc > 4)
    dispn = atoi(argv[4]);

  NavFn *nav;

  // try reading in a file
  int sx,sy;
  COSTTYPE *cmap = NULL;
  cmap = readPGM("willow-full-0.05.pgm",&sx,&sy);
  if (cmap)
    {
      nav = new NavFn(sx,sy);
      // find goal
      COSTTYPE *cm = cmap + sy/2 * sx + sx - 10;
      for (int i=0; i<sx-20; i++, cm--)
	{
	  if (*cm == COST_NEUTRAL)
	    {
	      goal[0] = sx-10-i;
	      printf("[NavTest] Found goal at X = %d\n", sx - 10 -i);
	      break;
	    }
	}
      goal[1] = sy/2;
    }
  else
    {
      sx = (int)((.001 + size) / (res*.001));
      sy = sx;
      nav = new NavFn(sx,sy); // size in pixels
      goal[0] = sx-10;
      goal[1] = sy/2;
    }

  // display
  nwin = new NavWin(sx,sy,"Potential Field");
  nwin->maxval = 2*sx*COST_NEUTRAL;
  Fl::visual(FL_RGB);
  nwin->show();


  // set goal and robot poses
  int *gg = goal;
  nav->setGoal(gg);

  // set display function
  nav->display(dispPot,dispn);

  // set up cost map from file, if it exists
  if (cmap)
    {
      nav->costarr = cmap;
      nav->setupNavFn(true);
    }
  else
    {
      nav->setupNavFn(false);
      nav->setObs();		// simple obstacles
    }

  // calculate the nav fn and path
  nav->priInc = inc;
  double t0 = get_ms();
  nav->propNavFn(sx*sy/20);
  double t1 = get_ms();

  printf("Time for plan calculation: %d ms\n", (int)(t1-t0));
  
  // draw potential field
  float mmax = 0.0;
  float *pp = nav->potarr;
  int ntot = 0;
  for (int i=0; i<nav->ny*nav->nx; i++, pp++)
    {
      if (*pp < 10e7 && *pp > mmax)
	mmax = *pp;
      if (*pp > 10e7)
	ntot++;			// number of uncalculated cells
    }
  printf("[NavFn] Cells not touched: %d/%d\n", ntot, nav->nx*nav->ny);
  nwin->maxval = 4*mmax/3;
  dispPot(nav);
  while (Fl::check()) {}

#if 0
  goal[1] = size-2;
  int k = nav->getCellIndex(gg);
  int st_nx = nav->st_nx;
  for (int i=0; i<900; i++, k--)
    {
      float npot = nav->potgrid[k];
      printf("Pot: %0.1f\n", npot);
      printf("L: %0.1f R: %0.1f U: %0.1f D: %0.1f\n",
	     nav->potgrid[k-1], nav->potgrid[k+1], nav->potgrid[k-st_nx], nav->potgrid[k+st_nx]);
    }
#endif

  return 0;
}


// read in a PGM file for obstacles
// no expansion yet...

static int CS;

void
setcostobs(COSTTYPE *cmap, int n, int w)
{
  CS = 11;
  for (int i=-CS/2; i<CS/2; i++)
    {
      COSTTYPE *cm = i*w + &cmap[n];
      for (int j=-CS/2; j<CS/2; j++)
	cm[j] = COST_NEUTRAL + 50;
    }
  CS = 7;
  for (int i=-CS/2; i<CS/2; i++)
    {
      COSTTYPE *cm = i*w + &cmap[n];
      for (int j=-CS/2; j<CS/2; j++)
	cm[j] = COST_OBS;
    }
}

void setcostunk(COSTTYPE *cmap, int n, int w)
{
  cmap[n] = COST_OBS;
}

COSTTYPE *
readPGM(char *fname, int *width, int *height)
{
  int fake_argc(1);
  char * fake_arg("foo");
  pgm_init(&fake_argc, &fake_arg);

  FILE *pgmfile;
  pgmfile = fopen(fname,"r");
  if (!pgmfile)
    {
      printf("[NavTest] Can't find file %s\n", fname);
      return NULL;
    }

  printf("[NavTest] Reading costmap file %s\n", fname);
  int ncols, nrows;
  gray maxval;
  int format;
  pgm_readpgminit(pgmfile, &ncols, &nrows, &maxval, &format);
  printf("[NavTest] Size: %d x %d\n", ncols, nrows);

  // set up cost map
  COSTTYPE *cmap = (COSTTYPE *)malloc(ncols*nrows*2);
  for (int i=0; i<ncols*nrows; i++)
    cmap[i] = COST_NEUTRAL;

  gray * row(pgm_allocrow(ncols));
  int otot = 0;
  int utot = 0;
  for (int ii(nrows - 1); ii >= 0; --ii) {
    pgm_readpgmrow(pgmfile, row, ncols, maxval, format);
    for (int jj(ncols - 1); jj >= 0; --jj)
      {
	if (row[jj] < unknown_gray && ii < nrows-7 && ii > 7)
	  {
	    setcostobs(cmap,ii*ncols+jj,ncols);
	    otot++;
	  }
#if 1
	else if (row[jj] <= unknown_gray)
	  {
	    setcostunk(cmap,ii*ncols+jj,ncols);
	    utot++;
	  }
#endif
      }
  }
  printf("[NavTest] Found %d obstacle cells, %d unknown cells\n", otot, utot);
  pgm_freerow(row);
  *width = ncols;
  *height = nrows;
  return cmap;
}
