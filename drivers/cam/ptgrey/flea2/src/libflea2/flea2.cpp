#include "flea2/flea2.h"
#include <stdexcept>


/*
void save_image(int num)
{
  uint32_t csize = jw.compress_to_jpeg((uint8_t *)dc1394Camera.capture_buffer, dc1394Camera.frame_width, dc1394Camera.frame_height);
  char fn[200];
  snprintf(fn, 200, "image%06d.jpg", num);
  FILE *imagefile = fopen(fn, "w" );
  fwrite(jw.get_compress_buf(), 1, csize, imagefile);
  fclose(imagefile);
}
*/

Flea2::Flea2(video_mode_t _video_mode, int n_host) :
  video_mode(_video_mode), raw_created(false), 
  cam_created(false), frame_released(true)
{
  raw = dc1394_create_handle(n_host);
  if (!raw)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle\n\n"
      "Please check \n"
      "if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded\n"
      "if you have read/write access to /dev/raw1394\n" );
    throw std::runtime_error("toast");
  }
  raw_created = true;
  nodeid_t *nodes;
  int num_cams, num_nodes;
  num_nodes = raw1394_get_nodecount(raw);
  nodes = dc1394_get_camera_nodes(raw, &num_cams, 1);
  if (num_cams < 1)
  {
    fprintf( stderr, 
        "No cameras found! (%d nodes on the bus)\n"
        "  - could be you need to try a different 1394 device (modify code to fix)\n", 
        num_nodes);
    throw std::runtime_error("woah");
  }
  printf("Working with the first camera on the bus\n");
  // skip error checking
  cam_node = nodes[0];
  if(cam_node == num_nodes-1)
  {
    fprintf(stderr, "the camera is the root node. that's bad.\n");
    throw std::runtime_error("woah");
  }

  unsigned int channel, speed;
  if (dc1394_get_iso_channel_and_speed(raw, cam_node, &channel, &speed)
      != DC1394_SUCCESS)
  {
    fprintf(stderr, "Unable to get the iso channel number\n" );
    throw std::runtime_error("toast");
  }

  dc1394_set_operation_mode(raw, cam_node, OPERATION_MODE_1394B);

  char host_dev[200];
  snprintf(host_dev, sizeof(host_dev), "/dev/video1394/%d", n_host);
  int e;
  if (video_mode == FLEA2_MONO)
    e = dc1394_dma_setup_capture(raw, cam_node, channel, 
          FORMAT_VGA_NONCOMPRESSED, MODE_640x480_MONO, SPEED_800,
          FRAMERATE_30, 8, 1, host_dev, &cam);
  else
    e = dc1394_dma_setup_capture(raw, cam_node, channel, 
          FORMAT_VGA_NONCOMPRESSED, MODE_640x480_RGB, SPEED_800,
          FRAMERATE_30, 8, 1, host_dev, &cam);
  if ( e != DC1394_SUCCESS )
  {
    fprintf(stderr, "Unable to setup camera-\n"
        "check code above line %d of %s to make sure\n"
        "that the video mode,framerate and format are\n"
        "supported by your camera\n",
        __LINE__,__FILE__);
    throw std::runtime_error("aaaa");
  }
  cam_created = true;
  printf("Starting iso transmission...\n" );
  if (dc1394_start_iso_transmission(raw, cam.node) != DC1394_SUCCESS)
  {
    fprintf(stderr, "Unable to start camera iso transmission\n" );
    throw std::runtime_error("asdf");
  }
}

Flea2::~Flea2()
{
  if (cam_created)
  {
    dc1394_stop_iso_transmission(raw, cam.node);
    dc1394_dma_unlisten(raw, &cam);
    dc1394_dma_release_camera(raw, &cam);
  }
  if (raw_created)
    raw1394_destroy_handle(raw);
}

bool Flea2::get_frame(uint8_t ** const frame, uint32_t *width, uint32_t *height)
{
  if (!frame_released)
    release_frame(); // maybe the coder forgot to do it.
  if (dc1394_dma_single_capture(&cam) == DC1394_SUCCESS) 
  {
    *frame  = (uint8_t *)cam.capture_buffer;
    *width  = cam.frame_width;
    *height = cam.frame_height;
    frame_released = false;
    return true;
  }
  else
  {
    *frame = NULL;
    *width = *height = 0;
    return false;
  }
}

void Flea2::release_frame()
{
  if (frame_released)
    return; // nothing to do
  dc1394_dma_done_with_buffer(&cam);
  frame_released = true;
}

bool Flea2::get_jpeg(const uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size)
{
  if (dc1394_dma_single_capture(&cam) != DC1394_SUCCESS) 
  {
    printf("couldn't get an image from the camera.\n");
    return false;
  }
  *fetch_buf_size = jpeg.compress_to_jpeg((uint8_t *)cam.capture_buffer, 
                                          cam.frame_width, 
                                          cam.frame_height,
                                          JpegWrapper::RT_MONO8,
                                          99);
  *fetch_jpeg_buf = jpeg.get_compress_buf();
  dc1394_dma_done_with_buffer(&cam);
  return true;
}
  
void Flea2::set_shutter(double d)
{
  if (d < 0) d = 0;
  else if (d > 1) d = 1;
  dc1394_set_shutter(raw, cam_node, (int)(d * 639));
}

void Flea2::set_gamma(double g)
{
  if (g < 0) g = 0;
  else if (g > 1) g = 1;
  dc1394_set_gamma(raw, cam_node, (int)(512 + g * 3500));
}

void Flea2::set_gain(double g)
{
  if (g < 0) g = 0;
  else if (g > 1) g = 1;
  dc1394_set_gain(raw, cam_node, (int)(g * 682));
}

