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

#include "sbpl_util.hh"
#include <costmap_2d/costmap_2d.h>

// for sbpl... not a greatly descriptive name
#include <headers.h>

#include <iostream>
#include <err.h>

using namespace ompl;
using namespace costmap_2d;
using namespace std;

static int const width(5);
static int const height(5);
static double const startx(0);
static double const starty(0);
static double const starttheta(0);
static double const goalx(0);
static double const goaly(0);
static double const goaltheta(0);
static double const goaltol_x(0.3);
static double const goaltol_y(0.3);
static double const goaltol_theta(0.3);
static double const nominalvel_mpersecs(1);
static double const timetoturn45degsinplace_secs(1);

static bool gtObstacle(int ix, int iy)
{
  return ix == iy + 1;
}

template<typename array_t, typename value_t>
void initMapdata(array_t & mapdata, value_t obstcost)
{
  cout << "=============================\n"
       << " * means obstacle (cost = " << (int) obstcost << ")\n"
       << " . means freespace (cost = 0)\n"
       << "-----------------------------\n";
  for (int ix(0); ix < width; ++ix) {
    cout << "  ";
    for (int iy(0); iy < height; ++iy)
      if (gtObstacle(ix, iy)) {
	cout << "*";
	mapdata[iy * width + ix] = obstcost;
      }
      else {
	cout << ".";
	mapdata[iy * width + ix] = 0;
      }
    cout << "\n";
  }
  cout << "=============================\n";
}

template<typename outline_t>
void initOutline(outline_t & outline)
{
  outline.resize(3);
  outline[0].x = 0;
  outline[0].y = 0;
  outline[1].x = 1;
  outline[1].y = 0;
  outline[2].x = 0;
  outline[2].y = 1;
}

template<typename env_t>
static bool check(env_t & env)
{
  cout << "===========================\n"
       << " * means correct obstacle\n"
       << " . means correct freespace\n"
       << " O means spurious freespace\n"
       << " x means spurious obstacle\n"
       << "---------------------------\n";
  bool ok(true);
  for (unsigned int ix(0); ix < width; ++ix) {
    cout << "  ";
    for (unsigned int iy(0); iy < height; ++iy)
      if (gtObstacle(ix, iy)) {
	if (env.IsObstacle(ix, iy))
	  cout << "*";		// obstacle in both
	else {
	  ok = false;
	  cout << "O";		// obstacle missing in env
	}
      }
      else {
	if (env.IsObstacle(ix, iy)) {
	  ok = false;
	  cout << "x";		// extra obstacle in env
	}
	else
	  cout << ".";		// freespace in both
      }
    cout << "\n";
  }
  if (ok)
    cout << "---------------------------\n"
	 << "check passed\n"
	 << "===========================\n";
  else
    cout << "---------------------------\n"
	 << "check FAILED\n"
	 << "===========================\n";
  return ok;
}

template<typename env_t, typename value_t>
bool updateCost(env_t & env, value_t obstcost)
{
  bool ok(true);
  for (unsigned int ix(0); ix < width; ++ix) {
    cout << "  ";
    for (unsigned int iy(0); iy < height; ++iy) {
      int cost(0);
      if (gtObstacle(ix, iy)) {
	cost = obstcost;
	cout << "*";
      }
      else
	cout << ".";
      if ( ! env.UpdateCost(ix, iy, cost)) {
	ok = false;
	cout << "ERROR ";
      }
    }
    cout << "\n";
  }
  if (ok)
    cout << "update passed\n";
  else
    cout << "update FAILED\n";
  return ok;
}

template<typename env_t, typename cost_t>
bool runTests(env_t & env, cost_t obstcost)
{
  bool ok(true);
  cout << "After init:\n";
  if ( ! check(env))
    ok = false;
  cout << "\nExplicit UpdateCost():\n";
  if ( ! updateCost(env, obstcost))
    ok = false;
  cout << "After explicit UpdateCost():\n";
  if ( ! check(env))
    ok = false;
  return ok;
}

static bool checkNonWrap()
{
  static double const cellsize_m(1);
  
  vector<sbpl_2Dpt_t> perimeterptsV;
  initOutline(perimeterptsV);
  
  unsigned char mapdata[width * height];
  unsigned char obstcost(1);
  initMapdata(mapdata, obstcost);
  
  EnvironmentNAV3DKIN env3d;
  env3d.InitializeEnv(width, height, mapdata, startx, starty, starttheta,
		      goalx, goaly, goaltheta, goaltol_x, goaltol_y, goaltol_theta,
		      perimeterptsV, cellsize_m, nominalvel_mpersecs,
		      timetoturn45degsinplace_secs);
  
  return runTests(env3d, obstcost);
}

static bool checkWrap()
{
  static unsigned char const obst_cost_thresh(CostMap2D::LETHAL_OBSTACLE);
  static double const resolution(0.1);
  static double const window_length(0.1);

  EnvironmentWrapper3DKIN::footprint_t footprint;
  initOutline(footprint);
  
  std::vector<unsigned char> data(width * height);
  initMapdata(data, obst_cost_thresh);
  
  CostMap2D costmap(width, height, data, resolution, window_length, obst_cost_thresh);
  EnvironmentWrapper3DKIN envWrap(costmap, obst_cost_thresh, startx, starty, starttheta,
				  goalx, goaly, goaltheta, goaltol_x, goaltol_y, goaltol_theta,
				  footprint, nominalvel_mpersecs, timetoturn45degsinplace_secs);
  return runTests(envWrap, obst_cost_thresh);
}

int main(int agrc, char ** argv)
{
  bool ok(true);
  cout << "**************************************************\n"
       << "non-wrapped\n";
  if ( ! checkNonWrap())
    ok = false;
  cout << "**************************************************\n"
       << "wrapped\n";
  if ( ! checkWrap())
    ok = false;
  cout << "**************************************************\n";
  if ( ! ok)
    errx(EXIT_FAILURE, "something went wrong");
  exit(EXIT_SUCCESS);
}
