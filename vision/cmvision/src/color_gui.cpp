#include <unistd.h>

#include "opencv/cv.h"
#include "opencv_latest/CvBridge.h"

#include "color_gui.h"

ColorGui::ColorGui(ros::NodeHandle *nh)
  : Fl_Window(0,0,400,400) 
{
  Fl::scheme("plastic");

  this->imageBox = new Fl_Box(5,5,this->w()-10, this->h()-10);
  this->imageBox->box(FL_FLAT_BOX);
  this->imageBox->color(FL_BACKGROUND_COLOR);

  this->end();
  this->show();

  this->nodeHandle = nh;

  // Subscribe to an image stream
  this->nodeHandle->subscribe("image", 1, &ColorGui::imageCB, this );
}

ColorGui::~ColorGui()
{
}

void ColorGui::Run()
{
  while (true)
  {
    usleep(1000);
    Fl::check();
    ros::spinOnce();
  }
}

////////////////////////////////////////////////////////////////////////////////
// The image callback
void ColorGui::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage *cvImage;
  CvSize size;
  sensor_msgs::CvBridge imageBridge;

  const sensor_msgs::Image img = *msg;

  // Get the image as and RGB image
  imageBridge.fromImage(img, "rgb8");
  cvImage = imageBridge.toIpl();

  size = cvGetSize(cvImage);

  printf("Got image\n");

  // this shouldn't change often
  /*if ( (size.width != this->width) || (size.height != this->height))
  {
    this->width = size.width;
    this->height = size.height;

    if (this->uyvyImage)
      delete [] this->rgbImage;
    this->rgbImage = new uint8_t[this->width * this->height * 3];
  }*/

//(unsigned char *)cvImage->imageData


}
