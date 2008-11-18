#include <cassert>
#include <cstdlib>
#include <list>
#include <cstring>
#include "dc1394/dc1394.h"
#include "ros/time.h"
#include "cam_dc1394.h"

using std::list;
using namespace borg;

CamDC1394::CamDC1394() : dc(NULL), cam(NULL)
{
}

CamDC1394::~CamDC1394()
{
}


bool CamDC1394::_takePhoto(ImageSize size, uint8_t *raster)
{
  return true;
}

bool CamDC1394::_shutdown()
{
  if (cam)
    dc1394_camera_free(cam);
  if (dc)
    dc1394_free(dc);
  return true;
}

#define ENSURE(EXP) \
  do { \
    if (EXP != DC1394_SUCCESS) { \
      printf("libdc1394v2 error during " #EXP ", line %d, file %s\n", \
             __LINE__, __FILE__); \
      config_status = CAM_ERROR; \
      return false; \
    } \
  } while (0)

#define ENSURE_NOTNULL(EXP) \
  do { \
    if (!EXP) { \
      printf("unexpected null: " #EXP ", line %d, file %s\n", \
             __LINE__, __FILE__); \
      config_status = CAM_ERROR; \
      return false; \
    } \
  } while (0)

bool CamDC1394::_init()
{
  dc = dc1394_new();
  dc1394camera_list_t *cams = NULL;
  ENSURE(dc1394_camera_enumerate(dc, &cams));
  ENSURE_NOTNULL(cams);
  printf("%d cameras\n", cams->num);
  if (cams->num == 0)
  {
    printf("bogus. no 1394 cameras found. Resetting the bus...\n");
    // maybe the bus was hosed. reset it so perhaps it will work next time.
    dc1394_free(dc);
    return false;
  }
  for (uint32_t i = 0; i < cams->num; i++)
    printf("cam %d: %llu-%u\n", i, cams->ids[i].guid, cams->ids[i].unit);
  cam = dc1394_camera_new(dc, cams->ids[0].guid);
  dc1394_camera_free_list(cams);
  ENSURE_NOTNULL(cam);

  dc1394_camera_print_info(cam, stdout);
  ENSURE(dc1394_video_set_operation_mode(cam, DC1394_OPERATION_MODE_1394B));
  ENSURE(dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_800));
  ENSURE(dc1394_video_set_mode(cam, DC1394_VIDEO_MODE_FORMAT7_0));
  ENSURE(dc1394_format7_set_roi(cam, DC1394_VIDEO_MODE_FORMAT7_0,
                                DC1394_COLOR_CODING_MONO8,
                                DC1394_USE_MAX_AVAIL,
                                0, 0, 640, 480));
  if (DC1394_SUCCESS != dc1394_capture_setup(cam, 8, 
                                             DC1394_CAPTURE_FLAGS_DEFAULT))
  {
    dc1394_reset_bus(cam);
    ENSURE(dc1394_video_set_operation_mode(cam, DC1394_OPERATION_MODE_1394B));
    ENSURE(dc1394_video_set_iso_speed(cam, DC1394_ISO_SPEED_800));
    ENSURE(dc1394_capture_setup(cam, 8, DC1394_CAPTURE_FLAGS_DEFAULT));
    ENSURE(dc1394_video_set_mode(cam, DC1394_VIDEO_MODE_FORMAT7_0));
    ENSURE(dc1394_format7_set_roi(cam, DC1394_VIDEO_MODE_FORMAT7_0,
                                  DC1394_COLOR_CODING_MONO8,
                                  DC1394_USE_MAX_AVAIL,
                                  0, 0, 640, 480));
  }
  /*
  ENSURE(dc1394_video_set_transmission(cam, DC1394_ON), "set transmission on");
  dc1394video_frame_t *frame = NULL;
  ros::Time t_start(ros::Time::now());
  const int NUM_FRAMES = 200;
  list<uint8_t *> frames;
  for (int i = 0; i < NUM_FRAMES; i++)
  {
    ENSURE(dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &frame),"cap");
    if (frame && 
        (frame->frames_behind > 1 || 
         dc1394_capture_is_frame_corrupt(cam, frame) == DC1394_TRUE))
    {
      //printf("corrupt frame\n");
      dc1394_capture_enqueue(cam, frame);
    }
    else
    {
      //printf("got frame\n");
      uint8_t *frame_copy = new uint8_t[640*480];
      memcpy(frame_copy, frame->image, 640*480);
      frames.push_back(frame_copy);
      dc1394_capture_enqueue(cam, frame);
    }
  }
  printf("turning off camera transmission...\n");
  ENSURE(dc1394_video_set_transmission(cam, DC1394_OFF), "stop ISO xfer");
  ENSURE(dc1394_capture_stop(cam), "capture stop");
  ros::Time t_end(ros::Time::now());
  double dt = t_end.to_double() - t_start.to_double();
  printf("average time = %f fps = %f\n", dt / NUM_FRAMES, NUM_FRAMES / dt);
  printf("%d frames saved\n", frames.size());
  int frame_count = 0;
  for (list<uint8_t *>::iterator f = frames.begin(); f != frames.end(); ++f)
  {
    char fnamebuf[100];
    snprintf(fnamebuf, sizeof(fnamebuf), "img%06d.pgm", frame_count++);
    FILE *file = fopen(fnamebuf, "w");
    assert(file);
    fprintf(file, "P5\n640 480\n255\n");
    fwrite(*f, 640*480, 1, file);
    fclose(file);
    delete[] *f;
  }
  frames.clear();
  */
  
  return 0;
}

