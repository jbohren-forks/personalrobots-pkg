///////////////////////////////////////////////////////////////////////////////
// This package provides a bridge to an XML-RPC server delivering stereo
// images on another computer (i.e. Windows) with which we currently have no
// other way of communicating, since ROS isn't windows-compatible yet.
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
#include <string>
#include "XmlRpc.h"
#include <stdexcept>
#include "image_utils/ppm_wrapper.h"
#include "image_utils/cv_bridge.h"
#include "opencv/highgui.h"
using namespace XmlRpc;

const char *BUMBLEBEE_HOST = "stair-vision";
const int   BUMBLEBEE_PORT = 11411; // a palindromic prime, naturally

class BumblebeeBridge : public ros::Node
{
public:
  BumblebeeBridge() : ros::Node("bumblebee_bridge")
  {
    advertiseService("stereo", &BumblebeeBridge::stereo);
  }
  bool stereo(std_srvs::PolledStereoCloud::request  &req,
              std_srvs::PolledStereoCloud::response &res)
  {
    printf("forwarding stereo cloud request\n");
    CvBridge<std_msgs::Image> cv_bridge(&res.image);
    XmlRpcClient c(BUMBLEBEE_HOST, BUMBLEBEE_PORT);
    XmlRpcValue noargs, result;
    try
    {
      if (c.execute("BumbleImage", noargs, result))
      {
        const XmlRpcValue::BinaryData &image = result[0];
        int num_pts = result[1][0];
        printf("image has %d bytes, cloud has %d points\n", 
            image.size(), num_pts);
        IplImage *cv_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
        for (size_t i = 0; i < 480; i++)
        {
          unsigned char *p = (unsigned char *)cv_image->imageData + 
                             i * cv_image->widthStep;
          for (size_t j = 0; j < 640; j++)
          {
            unsigned char *q = p + j * 3;
            *(q+2) = image[i*640*3+j*3  ];
            *(q+1) = image[i*640*3+j*3+1];
            *(q  ) = image[i*640*3+j*3+2];
          }
        }
        cv_bridge.from_cv(cv_image, 99);
        cvReleaseImage(&cv_image);
        const XmlRpcValue::BinaryData &cloud = result[1][1];
        uint8_t *buf = new uint8_t[cloud.size()];
        for (size_t i = 0; i < cloud.size(); ++i)
          buf[i] = cloud[i];
        float x, y, z;
        unsigned short row, col;
        res.set_points_size(num_pts);
        res.set_rows_size(num_pts);
        res.set_cols_size(num_pts);
        for (int i = 0; i < num_pts; i++)
        {
          x = *((float *)(buf + i*(3*4 + 2*2))); // 3 float, 2 unsigned short
          y = *((float *)(buf + i*(3*4 + 2*2) + 4));
          z = *((float *)(buf + i*(3*4 + 2*2) + 8));
          row = *((uint16_t *)(buf + i*(3*4 + 2*2) + 12));
          col = *((uint16_t *)(buf + i*(3*4 + 2*2) + 14));
          res.points[i].x = x;
          res.points[i].y = y;
          res.points[i].z = z;
          res.rows[i] = row;
          res.cols[i] = col;
        }
        delete[] buf;
      }
      else
      {
        printf("xmlrpc query didn't work. maybe the windows executable "
               "wasn't running?\n");
        return false;
      }
    }
    catch (const XmlRpc::XmlRpcException &e)
    {
      printf("xmlrpc exception: [%s]\n", e.getMessage().c_str());
      return false;
    }
    return true;
  }
};

void standalone()
{
  XmlRpcClient c(BUMBLEBEE_HOST, BUMBLEBEE_PORT);
  XmlRpcValue noargs, result;
  try
  {
    if (c.execute("BumbleImage", noargs, result))
    {
      const XmlRpcValue::BinaryData &image = result[0];
      int num_pts = result[1][0];
      printf("image has %d bytes, cloud has %d points\n", 
             image.size(), num_pts);
      uint8_t *raster = new uint8_t[image.size()];
      for (size_t i = 0; i < image.size(); ++i)
        raster[i] = image[i];
      PpmWrapper::write_file("image.ppm", 640, 480, "rgb24", raster);
      const XmlRpcValue::BinaryData &cloud = result[1][1];
      uint8_t *buf = new uint8_t[cloud.size()];
      for (size_t i = 0; i < cloud.size(); ++i)
        buf[i] = cloud[i];
      float x, y, z;
      unsigned short row, col;
      FILE *f = fopen("cloud.txt", "w");
      for (int i = 0; i < num_pts; i++)
      {
        x = *((float *)(buf + i*(3*4 + 2*2))); // 3 float, 2 unsigned short
        y = *((float *)(buf + i*(3*4 + 2*2) + 4));
        z = *((float *)(buf + i*(3*4 + 2*2) + 8));
        row = *((unsigned short *)(buf + i*(3*4 + 2*2) + 12));
        col = *((unsigned short *)(buf + i*(3*4 + 2*2) + 14));
        unsigned char r = *(raster + 640 * 3 * row + 3 * col    );
        unsigned char g = *(raster + 640 * 3 * row + 3 * col + 1);
        unsigned char b = *(raster + 640 * 3 * row + 3 * col + 2);
        fprintf(f, "%f %f %f %f %f %f\n", x, y, z, 
                r / 255.0, g / 255.0, b / 255.0);
      }
      fclose(f);
      delete[] raster;
      delete[] buf;
    }
    else
    {
      printf("xmlrpc query didn't work\n");
    }
  }
  catch (const XmlRpc::XmlRpcException &e)
  {
    printf("xmlrpc exception: [%s]\n", e.getMessage().c_str());
  }
}

int main(int argc, char **argv)
{
  printf("argc = %d\n", argc);
  if (argc > 1)
  {
    printf("entering standalone test\n");
    standalone();
    printf("done with standalone test\n");
  }
  else
  {
    printf("entering ROS node logic\n");
    ros::init(argc, argv);
    BumblebeeBridge b;
    b.spin();
    ros::fini();
  }
  return 0;
}

