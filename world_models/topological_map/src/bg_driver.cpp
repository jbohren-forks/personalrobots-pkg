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
#include "topological_map/bottleneck_graph.h"

using std::cout;
using std::endl;


int main (int argc, char* argv[])
{
  assert (argc>=4);
  topological_map::GridArray *grid;
  if ((argc==4) || (atoi(argv[4])==0)) {
  // Initialize grid
    grid = new topological_map::GridArray(boost::extents[4][5]);
    (*grid)[0][2] = true;
    (*grid)[2][2] = true;
    (*grid)[3][2] = true;
  }
  else {
    grid = new topological_map::GridArray(boost::extents[41][41]);
    for (int i=0; i<20; i++) {
      (*grid)[10][i] = true;
      (*grid)[10][40-i] = true;
    }
  }
      
  cout << "Bottleneck graph:" << endl;
  topological_map::IndexedBottleneckGraph g = topological_map::makeBottleneckGraph (*grid, atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
  g.printBottleneckGraph ();
  cout << "Bottlenecks:" << endl;
  g.printBottlenecks();
}

  
  
  
  
  
