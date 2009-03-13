/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <iostream>
#include <fstream>
#include <getopt.h>
#include <sysexits.h>
#include <topological_map/topological_map.h>

typedef unsigned int uint;
typedef const uint cuint;

using std::cout;
using std::endl;
using std::ifstream;
using boost::extents;
namespace tmap=topological_map;
using topological_map::OccupancyGrid;
using topological_map::TopologicalMapPtr;
using topological_map::topologicalMapFromGrid;
using topological_map::Point2D;
using topological_map::Cell2D;

void setV (topological_map::OccupancyGrid& grid, cuint r0, cuint dr, cuint rmax, cuint c0, cuint dc, cuint cmax, bool val) 
{
  for (uint r=r0; r<rmax; r+=dr) {
    for (uint c=c0; c<cmax; c+=dc) {
      grid[r][c] = val;
    }
  }
}


int main (int argc, char* argv[])
{
  
  OccupancyGrid grid(extents[21][24]);
  setV(grid, 0, 1, 21, 0, 1, 24, false);
  setV(grid, 7, 7, 21, 0, 1, 24, true);
  setV(grid, 0, 1, 21, 8, 8, 24, true);
  setV(grid, 3, 7, 21, 8, 8, 24, false);
  setV(grid, 7, 7, 21, 4, 8, 24, false);
  
  for (uint r=0; r<21; r++) {
    for (uint c=0; c<24; c++) {
      if (grid[r][c]) cout << "X"; else cout << ".";
    }
    cout << endl;
  }
  

  TopologicalMapPtr m = topologicalMapFromGrid (grid, 1.0, 100.0, 2, 1, 1, 0, "local");

  cout << *m;

  std::ofstream str("local/test");
  m->writeToStream(str);
  std::ofstream str2("local/out.ppm");
  m->writePpm(str2);
  m->connectorCosts(Point2D(1,1), Point2D(10,10));

//   std::ifstream str3("local/willow.tmap");
//   tmap::TopologicalMap m2(str3);
//   std::ofstream str4("local/willow2.tmap");
//   m2.writeToStream(str4);
//   std::ofstream str5("local/willow2.ppm");
//   m2.writePpm(str5);
}

  
  
  
  
  
