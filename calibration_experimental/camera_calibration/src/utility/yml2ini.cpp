#include "camera_calibration/pinhole.h"
#include <ros/console.h>

using namespace camera_calibration;

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s input.yml output.ini\n", argv[0]);
    return 0;
  }

  PinholeCameraModel model;
  if (!model.load(argv[1])) {
    ROS_ERROR("Failed to load camera model from file %s", argv[1]);
    return -1;
  }
  if (!model.save(argv[2])) {
    ROS_ERROR("Failed to save camera model to file %s", argv[2]);
  }
  
  ROS_INFO("Saved %s", argv[2]);
  return 0;
}
