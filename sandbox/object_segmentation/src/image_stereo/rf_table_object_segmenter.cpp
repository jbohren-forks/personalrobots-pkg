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

#include <object_segmentation/image_stereo/table_object_rf.h>

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char *argv[])
{
  TableObjectRF table_object_rf;
  /*
  if (argc != 3)
  {
    ROS_WARN("%s usage: <file list> <output dir/>", argv[0]);
    return -1;
  }

  ifstream file_list(argv[1]);
  if (file_list.is_open() == false)
  {
    ROS_FATAL("Could not open file %s", argv[1]);
    return -1;
  }

  // yaw pitch roll
  // base-path
  // [filenames]
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  file_list >> yaw;
  file_list >> pitch;
  file_list >> roll;
  char base_path[256];
  file_list >> base_path;
  while (file_list.eof() == false)
  {
    // basename = filename without .extension
    char basename[256];
    file_list >> basename;

    // full_path/basename-LR.png
    string img_fname(base_path);
    img_fname.append(basename);
    img_fname.append("-LR.png");

    // full_path/basename.pcd_dan
    string pcd_fname(base_path);
    pcd_fname.append(basename);
    pcd_fname.append(".pcd_dan");

    boost::shared_ptr<RandomField> rf = table_object_rf.createRandomField(img_fname, pcd_fname,
        yaw, pitch, roll);

    // out_path/basename (.random-field)
    string rf_fname(argv[2]);
    rf_fname.append(basename);
    if (rf->saveRandomField(rf_fname) < 0)
    {
      abort();
    }
  }
*/
  string path("/u/msun/data/texture_light_3d_dataset/stapler/stapler_8");
  string fname_pcd = path;
  fname_pcd.append("/stapler_8_A8_H1_S1.pcd_dan");
  string fname_img = path;
  fname_img.append("/stapler_8_A8_H1_S1-LR.png");

  boost::shared_ptr<RandomField> rf = table_object_rf.createRandomField(fname_img, fname_pcd, 0, 0,
      1.9);
  rf->saveNodeFeatures("the_nodes");

  return 0;
}
