#ifndef UVC_CAM_H
#define UVC_CAM_H

namespace uvc_cam
{

class Cam
{
public:
  Cam() { }
  ~Cam() { }
  static void enumerate();
};

}

#endif

