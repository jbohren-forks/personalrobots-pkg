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

#include "image_msgs/CvBridge.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/Image.h"

#include <boost/thread.hpp>

class ProsilicaView : public ros::Node
{
private:
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  IplImage* image_;

  boost::mutex cv_mutex_;

public:
  ProsilicaView() : ros::Node("prosilica_view"), image_(NULL)
  {
    cvNamedWindow("Prosilica", CV_WINDOW_AUTOSIZE);

    subscribe(mapName("prosilica") + "/image_rect", img_msg_,
              &ProsilicaView::image_cb, this, 1);
  }

  ~ProsilicaView()
  {
    cvReleaseImage(&image_);
  }

  void image_cb()
  {
    boost::lock_guard<boost::mutex> guard(cv_mutex_);

    if (img_bridge_.fromImage(img_msg_, "bgr"))
      cvShowImage("Prosilica", img_bridge_.toIpl());
  }
  
  bool spin()
  {
    while (ok())
    {
      cv_mutex_.lock();
      int key = cvWaitKey(3);
      
      //switch (key) {
      //}

      cv_mutex_.unlock();
      usleep(10000);
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ProsilicaView view;
  view.spin();
  
  return 0;
}

