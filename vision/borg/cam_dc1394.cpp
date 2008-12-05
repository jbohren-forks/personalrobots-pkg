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
  printf("camdc1394 init\n");
  dc = dc1394_new();
  dc1394camera_list_t *cams = NULL;
  ENSURE(dc1394_camera_enumerate(dc, &cams));
  ENSURE_NOTNULL(cams);
  if (cams->num == 0)
  {
    printf("bogus. no 1394 cameras found.\n");
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
  uint32_t sh1, sh2, g1, g2, ga1, ga2, b1, b2;
  dc1394_feature_get_boundaries(cam, DC1394_FEATURE_SHUTTER, &sh1, &sh2);
  printf("shutter range: %d to %d\n", sh1, sh2);
  dc1394_feature_get_boundaries(cam, DC1394_FEATURE_GAIN, &g1, &g2);
  printf("gain range: %d to %d\n", g1, g2);
  dc1394_feature_get_boundaries(cam, DC1394_FEATURE_GAMMA, &ga1, &ga2);
  printf("gamma range: %d to %d\n", ga1, ga2);
  dc1394_feature_get_boundaries(cam, DC1394_FEATURE_BRIGHTNESS, &b1, &b2);
  printf("brightness range: %d to %d\n", b1, b2);
  /*
  printf("turning off camera transmission...\n");
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
  
  return true;
}

bool CamDC1394::_startImageStream()
{
  ENSURE(dc1394_video_set_transmission(cam, DC1394_ON));
  return true;
}

bool CamDC1394::_savePhoto(uint8_t *photo)
{
  int attempt = 0;
  do
  {
    dc1394video_frame_t *frame = NULL;
    ENSURE(dc1394_capture_dequeue(cam, DC1394_CAPTURE_POLICY_WAIT, &frame));
    if (frame && 
        (frame->frames_behind > 1))// || 
         //dc1394_capture_is_frame_corrupt(cam, frame) == DC1394_TRUE))
      dc1394_capture_enqueue(cam, frame);
    else
    {
      memcpy(photo, frame->image, 640*480);
      dc1394_capture_enqueue(cam, frame);
      return true;
    }
  } while (attempt++ < 10);
  return false;

  //ros::Time t_start(ros::Time::now());
  //const int NUM_FRAMES = 200;
  //list<uint8_t *> frames;
  //for (int i = 0; i < NUM_FRAMES; i++)
  //{
  //}
}

bool CamDC1394::_stopImageStream()
{
  ENSURE(dc1394_video_set_transmission(cam, DC1394_OFF));
  ENSURE(dc1394_capture_stop(cam));
  return true;
}

bool CamDC1394::set(const char *setting, uint32_t value)
{
  printf("setting %s to %d\n", setting, value);
  if (!strcmp(setting, "shutter"))
    dc1394_feature_set_value(cam, DC1394_FEATURE_SHUTTER, value);
  else if (!strcmp(setting, "gain"))
    dc1394_feature_set_value(cam, DC1394_FEATURE_GAIN, value);
  else if (!strcmp(setting, "gamma"))
    dc1394_feature_set_value(cam, DC1394_FEATURE_GAMMA, value);
  else if (!strcmp(setting, "brightness"))
    dc1394_feature_set_value(cam, DC1394_FEATURE_BRIGHTNESS, value);
  return true;
}

