/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Range routines
 * Author: Andrew Howard
 * Date: 18 Jan 2003
 * CVS: $Id: map_range.c 1347 2003-05-05 06:24:33Z inspectorg $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "map.h"

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range)
#if 0
{
  int i, j;
  int ai, aj, bi, bj;
  double dx, dy;
  map_cell_t *cell;
  
  if (fabs(cos(oa)) > fabs(sin(oa)))
  {
    ai = MAP_GXWX(map, ox);
    bi = MAP_GXWX(map, ox + max_range * cos(oa));
    
    aj = MAP_GYWY(map, oy);
    dy = tan(oa) * map->scale;

    if (ai < bi)
    {
      for (i = ai; i < bi; i++)
      {
        j = MAP_GYWY(map, oy + (i - ai) * dy);
        if (MAP_VALID(map, i, j))
        {
          cell = map->cells + MAP_INDEX(map, i, j);
          if (cell->occ_state >= 0)
            return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
        }
        else
            return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;          
      }
    }
    else
    {
      for (i = ai; i > bi; i--)
      {
        j = MAP_GYWY(map, oy + (i - ai) * dy);
        if (MAP_VALID(map, i, j))
        {
          cell = map->cells + MAP_INDEX(map, i, j);
          if (cell->occ_state >= 0)
            return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
        }
        else
          return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
      }
    }
  }
  else
  {
    ai = MAP_GXWX(map, ox);
    dx = tan(M_PI/2 - oa) * map->scale;
    
    aj = MAP_GYWY(map, oy);
    bj = MAP_GYWY(map, oy + max_range * sin(oa));

    if (aj < bj)
    {
      for (j = aj; j < bj; j++)
      {
        i = MAP_GXWX(map, ox + (j - aj) * dx);
        if (MAP_VALID(map, i, j))
        {
          cell = map->cells + MAP_INDEX(map, i, j);
          if (cell->occ_state >= 0)
            return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
        }
        else
          return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
      }
    }
    else
    {
      for (j = aj; j > bj; j--)
      {
        i = MAP_GXWX(map, ox + (j - aj) * dx);
        if (MAP_VALID(map, i, j))
        {
          cell = map->cells + MAP_INDEX(map, i, j);
          if (cell->occ_state >= 0)
            return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
        }
        else
          return sqrt((i - ai) * (i - ai) + (j - aj) * (j - aj)) * map->scale;
      }
    }
  }
  return max_range;
}
#else
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map,ox);
  y0 = MAP_GYWY(map,oy);
  
  x1 = MAP_GXWX(map,ox + max_range * cos(oa));
  y1 = MAP_GYWY(map,oy + max_range * sin(oa));

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
  }
  else
  {
    if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
    else
    {
      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
  }
  return max_range;
}
#endif
