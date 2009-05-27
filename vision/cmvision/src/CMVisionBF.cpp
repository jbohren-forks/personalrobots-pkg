/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *     Andrew Martignoni III
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Uses CMVision to retrieve the blob data
 */
// author Andy Martignoni III, Brian Gerkey, Brendan Burns, Ben Grocholsky, Brad Kratochvil

#include "opencv/highgui.h"

#include "CMVisionBF.h"

using namespace color_blob_track;

////////////////////////////////////////////////////////////////////////////////
// Constructor
CMVisionBF::CMVisionBF(ros::Node *_node)
{
  this->node = _node;
  this->uyvyImage = NULL;
  this->width = 0;
  this->height = 0;

  this->vision = new CMVision();

  this->blobCount = 0;

  this->blobMessage.blobCount = 0;

  // Get the color file. This defines what colors to track
  this->node->param("/cmvision/color_file",this->colorFilename,
                    std::string("colors.txt"));

  // Get the level of debug output
  this->node->param("/cmvision/debug_on",this->debugOn,true);

  // Mean shift stuff
  this->node->param("/cmvision/do_mean_shift", this->doMeanShift, false);
  this->node->param("/cmvision/spatial_radius_pix", this->spatialRadius, 2.0);
  this->node->param("/cmvision/color_radius_pix", this->colorRadius, 40.0);

  // Subscribe to an image stream
  this->node->subscribe("image", this->image, &CMVisionBF::imageCB, this, 1);

  // Advertise our blobs
  this->node->advertise("blobs",this->blobMessage, 1);

  if (this->debugOn)
    cvNamedWindow("Image");
}

////////////////////////////////////////////////////////////////////////////////

CMVisionBF::~CMVisionBF()
{
  if (this->vision) 
    delete this->vision;
}


////////////////////////////////////////////////////////////////////////////////
// The image callback
void CMVisionBF::imageCB()
{
  IplImage *cvImage;
  CvSize size;

  // Timing
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  ros::Time startt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);

      
  // Get the image as and RGB image
  this->imageBridge.fromImage(this->image,"bgr");
  cvImage = this->imageBridge.toIpl();

  size = cvGetSize(cvImage);

  // this shouldn't change often
  if (( size.width != this->width) || (size.height != this->height))
  {
    if( !(this->vision->initialize(size.width, size.height)))
    {
      this->width = this->height = 0;
      ROS_ERROR("Vision init failed.");
      return;
    }

    if (!this->colorFilename.empty())
    {
      if (!this->vision->loadOptions(this->colorFilename.c_str()))
      {
        ROS_ERROR("Error loading color file");
        return;
      }
    }
    else
    {
      ROS_ERROR("No color file given.  Use the \"mColorFile\" "
                "option in the configuration file.");
      return;
    }

    this->width = size.width;
    this->height = size.height;

    if (this->uyvyImage)
      delete [] this->uyvyImage;
    this->uyvyImage = new uint8_t[this->width * this->height * 2];

  }

  // Smooth the image
  if (this->doMeanShift) 
  {
    cvPyrMeanShiftFiltering( cvImage, cvImage, this->spatialRadius, 
                             this->colorRadius );
  }

  // Convert image to YUV color space
  rgb2uyvy((unsigned char *)cvImage->imageData, this->uyvyImage,
           this->width*this->height);

  // Find the color blobs
  if (!this->vision->processFrame(
        reinterpret_cast<image_pixel*>(this->uyvyImage)))
  {
    ROS_ERROR("Frame error.");
    return;
  }

  // Get all the blobs
  this->blobCount = 0;
  for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
  {
    // Get the descriptive color
    rgb c = this->vision->getColorVisual(ch);

    // Grab the regions for this color
    CMVision::region* r = NULL;

    for (r = this->vision->getRegions(ch); r != NULL; r = r->next)
    {
      // Resize the blob message
      if (this->blobCount >= this->blobMessage.blobs.size())
      {
        this->blobMessage.blobs.resize( this->blobMessage.blobs.size()+1 );
      }

      if (this->debugOn)
        cvRectangle(cvImage, cvPoint(r->x1, r->y1), 
                    cvPoint(r->x2, r->y2), CV_RGB(c.blue, c.green, c.red));

      this->blobMessage.blobs[this->blobCount].red    = c.red;
      this->blobMessage.blobs[this->blobCount].green  = c.green;
      this->blobMessage.blobs[this->blobCount].blue   = c.blue;
      this->blobMessage.blobs[this->blobCount].area   = r->area;
      this->blobMessage.blobs[this->blobCount].x      = rint(r->cen_x+.5);
      this->blobMessage.blobs[this->blobCount].y      = rint(r->cen_y+.5);
      this->blobMessage.blobs[this->blobCount].left   = r->x1;
      this->blobMessage.blobs[this->blobCount].right  = r->x2;
      this->blobMessage.blobs[this->blobCount].top    = r->y1;
      this->blobMessage.blobs[this->blobCount].bottom = r->y2;

      this->blobCount++;
    }
  }

  if (this->debugOn)
  {
    cvShowImage("Image", cvImage);
    cvWaitKey(3);
  }

  this->blobMessage.blobCount = this->blobCount;

  this->node->publish("blobs", this->blobMessage);

  // Timing
  gettimeofday(&timeofday,NULL);
  ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
  ros::Duration diff = endt-startt;
  ROS_DEBUG_STREAM_NAMED("cmvision", "Color Blob Detection duration " << diff.toSec() );
}
