/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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
#include <stdlib.h>

#include <object_segmentation/laser/room_object_rf.h>

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char *argv[])
{
  RoomObjectRF room_object_rf;

  if (argc != 3 && argc!= 4)
  {
    ROS_WARN("%s usage: <file list> <output dir/> (0/1: Use ONLY labeled points)", argv[0]);
    return -1;
  }

  int use_only_labeled = true;
  if (argc == 4)
  {
    use_only_labeled = atoi(argv[3]);
  }

  ifstream file_list(argv[1]);
  if (file_list.is_open() == false)
  {
    ROS_FATAL("Could not open file %s", argv[1]);
    return -1;
  }

  bool done = false;
  while (!done)
  {
    // inclue .pcd
    char pcd_fname[256];
    file_list >> pcd_fname;

    boost::shared_ptr<RandomField> rf = room_object_rf.createRandomField(pcd_fname, use_only_labeled);

    string str_pcd_fname(pcd_fname);
    size_t slash_loc = str_pcd_fname.rfind('/');
    if (slash_loc == string::npos)
    {
      slash_loc = 0;
    }
    size_t ext_loc = str_pcd_fname.rfind('.');

    size_t length = ext_loc - slash_loc;

    string basename = str_pcd_fname.substr(slash_loc, length);

    // out_path/basename (.random-field)
    string rf_fname(argv[2]);
    rf_fname.append(basename);
    if (rf->saveRandomField(rf_fname) < 0)
    {
      abort();
    }

    if (file_list.eof())
    {
      done = true;
    }
  }
  return 0;
}
