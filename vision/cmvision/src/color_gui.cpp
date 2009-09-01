#include <unistd.h>

#include <boost/thread/mutex.hpp>
#include <FL/fl_draw.H>

#include "opencv_latest/CvBridge.h"
#include "conversions.h"
#include "color_gui.h"

#define RGB2YUV(r, g, b, y, u, v)\
  y = (306*r + 601*g + 117*b)  >> 10;\
  u = ((-172*r - 340*g + 512*b) >> 10)  + 128;\
  v = ((512*r - 429*g - 83*b) >> 10) + 128;\
  y = y < 0 ? 0 : y;\
  u = u < 0 ? 0 : u;\
  v = v < 0 ? 0 : v;\
  y = y > 255 ? 255 : y;\
  u = u > 255 ? 255 : u;\
  v = v > 255 ? 255 : v


////////////////////////////////////////////////////////////////////////////////
// Constructor
ColorGui::ColorGui(ros::NodeHandle *nh)
  : Fl_Window(0,0,800,600) 
{
  Fl::scheme("plastic");
  Fl::visual(FL_RGB);

  Fl_Group *rgbGroup = new Fl_Group(10,490,300,80);

  this->rgbOutput[0] = new Fl_Value_Output(rgbGroup->x()+30, 
      rgbGroup->y()+2, 32, 20, "R" );
  this->rgbOutput[1] = new Fl_Value_Output(this->rgbOutput[0]->x()+60, 
      this->rgbOutput[0]->y(), 32, 20, "G");
  this->rgbOutput[2] = new Fl_Value_Output(this->rgbOutput[1]->x()+60, 
      this->rgbOutput[1]->y(), 32, 20, "B" );

  this->yuvOutput[0] = new Fl_Value_Output(rgbGroup->x()+30, 
      this->rgbOutput[0]->y()+this->rgbOutput[0]->h()+5, 32, 20, "Y" );
  this->yuvOutput[1] = new Fl_Value_Output(this->rgbOutput[0]->x()+60, 
      this->rgbOutput[0]->y()+this->rgbOutput[0]->h()+5, 32, 20, "U" );
  this->yuvOutput[2] = new Fl_Value_Output(this->rgbOutput[1]->x()+60, 
      this->rgbOutput[0]->y()+this->rgbOutput[0]->h()+5, 32, 20, "V" );

  this->yuvRangeOutput[0] = new Fl_Output(rgbGroup->x()+30, 
      this->yuvOutput[0]->y()+this->yuvOutput[0]->h()+5, 65, 20, "Y" );
  this->yuvRangeOutput[1] = new Fl_Output(this->yuvRangeOutput[0]->x()+85, 
      this->yuvOutput[0]->y()+this->yuvOutput[0]->h()+5, 65, 20, "U" );
  this->yuvRangeOutput[2] = new Fl_Output(this->yuvRangeOutput[1]->x()+85, 
      this->yuvOutput[0]->y()+this->yuvOutput[0]->h()+5, 65, 20, "V" );

  rgbGroup->end();

  this->resetButton = new Fl_Button(rgbGroup->x(), rgbGroup->y() + rgbGroup->h() + 5, 50, 20, "Reset" );
  this->resetButton->callback( &ColorGui::ResetCB, this);

  this->end();
  this->show();

  this->nodeHandle = nh;

  this->mutex = new boost::mutex();

  this->width = 0;
  this->height = 0;
  this->rgbImage = NULL;

  // Subscribe to an image stream
  this->subscriber = this->nodeHandle->subscribe("image", 1, &ColorGui::imageCB, this );

  this->vision = new CMVision();
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
ColorGui::~ColorGui()
{
  delete this->vision;

  if (this->rgbImage)
    delete [] this->rgbImage;

  if (this->uyvyImage)
    delete [] this->uyvyImage;
}

////////////////////////////////////////////////////////////////////////////////
// Run the gui
void ColorGui::Run()
{
  while (this->visible())
  {
    usleep(10);
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
  if (img.encoding == "rgb8")
    imageBridge.fromImage(img, "rgb8");
  else
    imageBridge.fromImage(img, "bgr8");
  cvImage = imageBridge.toIpl();

  size = cvGetSize(cvImage);

  {
    boost::mutex::scoped_lock(*this->mutex);
    if (this->width != size.width || this->height != size.height)
    {
      if (!this->rgbImage)
        delete [] this->rgbImage;
      this->rgbImage = new unsigned char[size.width * size.height*3];

      if (!this->uyvyImage)
        delete [] this->uyvyImage;
      this->uyvyImage = new unsigned char[size.width * size.height * 2];

      if( !(this->vision->initialize(size.width, size.height)))
      {
        this->width = this->height = 0;
        ROS_ERROR("Vision init failed.");
        return;
      }
    }

    this->width = size.width;
    this->height = size.height;

    memcpy(this->rgbImage, cvImage->imageData, this->width*this->height*3);

    // Convert image to YUV color space
    rgb2uyvy(this->rgbImage, this->uyvyImage, this->width*this->height);
  }

  fl_draw_image(this->rgbImage, 5, 5, this->width, this->height,3,0);

  // Find the color blobs
  if (!this->vision->processFrame(
        reinterpret_cast<image_pixel*>(this->uyvyImage)))
  {
    ROS_ERROR("Frame error.");
    return;
  }

  // Get all the blobs
  for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
  {
    // Get the descriptive color
    rgb c = this->vision->getColorVisual(ch);

    // Grab the regions for this color
    CMVision::region* r = NULL;

    for (r = this->vision->getRegions(ch); r != NULL; r = r->next)
    {
      fl_rect( r->x1, r->y1, r->x2 - r->x1, r->y2 - r->y1 );
      /*cvRectangle(cvImage, cvPoint(r->x1, r->y1), 
          cvPoint(r->x2, r->y2), CV_RGB(c.blue, c.green, c.red));
          */
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Handle a mouse click
void ColorGui::HandleMouseRelease()
{
  int px = this->mousePosX - 5;
  int py = this->mousePosY - 5;

  if (px >= 0 && px < this->width &&
      py >= 0 && py < this->height)
  {
    int r,g,b,y,u,v;

    boost::mutex::scoped_lock(*this->mutex);
    r = this->rgbImage[py*(this->width*3) + px * 3 + 0];
    g = this->rgbImage[py*(this->width*3) + px * 3 + 1];
    b = this->rgbImage[py*(this->width*3) + px * 3 + 2];

    RGB2YUV(r, g, b, y, u, v);

    this->rgbOutput[0]->value(r);
    this->rgbOutput[1]->value(g);
    this->rgbOutput[2]->value(b);

    this->yuvOutput[0]->value(y);
    this->yuvOutput[1]->value(u);
    this->yuvOutput[2]->value(v);

    int y_low, y_high, u_low, u_high, v_low, v_high;

    this->vision->getThreshold(0,y_low,y_high,u_low,u_high,v_low,v_high);
  
    if (y_low == 0 && y_high == 0) { y_low = y; y_high = y; }
    if (u_low == 0 && u_high == 0) { u_low = u; u_high = u; }
    if (v_low == 0 && v_high == 0) { v_low = v; v_high = v; }

    y_low = std::min(y,y_low);
    y_high = std::max(y,y_high);

    u_low = std::min(u,u_low);
    u_high = std::max(u,u_high);

    v_low = std::min(v,v_low);
    v_high = std::max(v,v_high);

    this->vision->setThreshold(0,y_low,y_high,u_low,u_high,v_low,v_high);

    std::stringstream stream1, stream2, stream3;
    stream1 << y_low << ":" << y_high;
    stream2 << u_low << ":" << u_high;
    stream3 << v_low << ":" << v_high;
    this->yuvRangeOutput[0]->value(stream1.str().c_str());
    this->yuvRangeOutput[1]->value(stream2.str().c_str());
    this->yuvRangeOutput[2]->value(stream3.str().c_str());
  }

  this->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Handle events
int ColorGui::handle(int event)
{
  bool handled = false;
  this->mousePosX = Fl::event_x();
  this->mousePosY = Fl::event_y();

  switch(event)
  {
    case FL_RELEASE:
      this->HandleMouseRelease();
      handled = true;
      break;
  }

  if (!handled)
    return Fl_Window::handle(event);
  else
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the color selection
void ColorGui::ResetCB( Fl_Widget *w, void *data)
{
  ColorGui *self = (ColorGui*)(data);
  self->vision->setThreshold(0,0,0,0,0,0,0);
  self->yuvRangeOutput[0]->value("");
  self->yuvRangeOutput[1]->value("");
  self->yuvRangeOutput[2]->value("");
  self->redraw();
}
