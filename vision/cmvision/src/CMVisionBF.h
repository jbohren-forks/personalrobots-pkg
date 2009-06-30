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

#include <ros/ros.h>
#include <ros/node.h>
#include <sensor_msgs/Image.h>

#include "opencv/cv.h"
#include "opencv_latest/CvBridge.h"

#include "cmvision/Blobs.h"
#include "color_calib.h"
#include "conversions.h"
#include "cmvision.h"
#include "capture.h"

#define CMV_NUM_CHANNELS CMV_MAX_COLORS
#define CMV_HEADER_SIZE 4*CMV_NUM_CHANNELS
#define CMV_BLOB_SIZE 16
#define CMV_MAX_BLOBS_PER_CHANNEL 10

#define DEFAULT_CMV_WIDTH CMV_DEFAULT_WIDTH
#define DEFAULT_CMV_HEIGHT CMV_DEFAULT_HEIGHT

namespace color_blob_track
{

  class CMVisionBF
  {
    /// \brief Constructor
    public: CMVisionBF(ros::Node *node);

    /// \brief Destructor
    public: virtual ~CMVisionBF();

    /// \brief Image callback
    public: void imageCB();

    private: ros::Node *node;
    private: sensor_msgs::Image image;
    private: sensor_msgs::CvBridge imageBridge;

    private: bool  debugOn;
    private: uint16_t width;
    private: uint16_t height;
    private: std::string colorFilename;
    private: uint8_t *uyvyImage;

    private: unsigned int blobCount;
  
    private: CMVision *vision;

    private: cmvision::Blobs blobMessage;

    private: bool meanShiftOn;
    private: double spatialRadius;
    private: double colorRadius;

    private: bool colorCalOn;
    private: color_calib::Calibration *colorCal;
  };
  
}
