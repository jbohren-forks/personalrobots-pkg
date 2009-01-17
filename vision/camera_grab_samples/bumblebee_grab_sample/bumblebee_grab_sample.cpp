///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include "ros/node.h"
#include "std_srvs/PolledStereoCloud.h"
#include "image_utils/cv_bridge.h"
#include "opencv/highgui.h"

class BumblebeeGrabSample : public ros::Node
{
public:
  BumblebeeGrabSample() : ros::Node("bumblebee_grab_sample")
  {
  }
  bool grab_image()
  {
    std_srvs::PolledStereoCloud::request  req;
    std_srvs::PolledStereoCloud::response res;
    CvBridge<std_msgs::Image> cv_bridge(&res.image);
    if (ros::service::call("stereo", req, res))
    {
      printf("success\n");
      printf("image is %d by %d\n", res.image.width, res.image.height);
      // for this sample, I'll just dump the image and point cloud to file.
      IplImage *cv_image;
      if (!cv_bridge.to_cv(&cv_image))
        return false; // this shouldn't happen, but avoid null pointers if so
      cvSaveImage("test.jpg", cv_image);
      cvReleaseImage(&cv_image);
      printf("writing point cloud file for %d points\n", res.get_points_size());
      FILE *f = fopen("cloud.txt", "w");
      for (size_t i = 0; i < res.get_points_size(); i++)
        fprintf(f, "%f %f %f %d %d\n", 
                res.points[i].x, res.points[i].y, res.points[i].z,
                res.rows[i], res.cols[i]);
      fclose(f);
      return true;
    }
    else
      return false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  BumblebeeGrabSample b;
  if (!b.grab_image())
    printf("failed to grab a bumblebee image. please make sure that the "
           "bumblebee_bridge node is running and that the xml-rpc standalone "
           "program is running on the windows box.\n");
  ros::fini();
  return 0;
}

