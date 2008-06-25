#include "fw_cam.h"
#include <stdexcept>

FwHost::FwHost(int n_host) : raw_created(false)
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

  snprintf(host_dev, sizeof(host_dev), "/dev/video1394/%d", n_host);
}

FwHost::~FwHost() {
  if (raw_created)
    raw1394_destroy_handle(raw);
}


FwCam::FwCam(FwHost* host, int n_dev, video_mode_t _video_mode) : 
  video_mode(_video_mode),  
  mp_host(host),
  frame_released(true),
  cam_created(false)
{
  if (n_dev > mp_host->num_nodes)
  {
    fprintf(stderr, "The fwHost doesn't have that many nodes!\n");
    throw std::runtime_error("woah");
  }

  cam_node = mp_host->nodes[n_dev];

  if(cam_node == mp_host->num_nodes-1)
  {
    fprintf(stderr, "the camera is the root node. that's bad.\n");
    throw std::runtime_error("woah");
  }

  unsigned int channel, speed;
  if (dc1394_get_iso_channel_and_speed(mp_host->raw, cam_node, &channel, &speed)
      != DC1394_SUCCESS)
  {
    fprintf(stderr, "Unable to get the iso channel number\n" );
    throw std::runtime_error("toast");
  }

  dc1394_set_operation_mode(mp_host->raw, cam_node, OPERATION_MODE_1394B);

  int e;
  if (video_mode == FLEA2_MONO)
    e = dc1394_dma_setup_capture(mp_host->raw, cam_node, channel, 
				 FORMAT_VGA_NONCOMPRESSED, MODE_640x480_MONO, SPEED_400,
				 //	  FORMAT_SVGA_NONCOMPRESSED_2, MODE_1280x960_MONO, SPEED_400,
          FRAMERATE_30, 8, 1, mp_host->host_dev, &cam);
  else
    e = dc1394_dma_setup_capture(mp_host->raw, cam_node, channel, 
          FORMAT_VGA_NONCOMPRESSED, MODE_640x480_RGB, SPEED_400,
          FRAMERATE_30, 8, 1, mp_host->host_dev, &cam);
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
  if (dc1394_start_iso_transmission(mp_host->raw, cam.node) != DC1394_SUCCESS)
  {
    fprintf(stderr, "Unable to start camera iso transmission\n" );
    throw std::runtime_error("asdf");
  }
}

FwCam::~FwCam()
{
  if (cam_created)
  {
    dc1394_stop_iso_transmission(mp_host->raw, cam.node);
    dc1394_dma_unlisten(mp_host->raw, &cam);
    dc1394_dma_release_camera(mp_host->raw, &cam);
  }

}

bool FwCam::get_frame(uint8_t ** const frame, uint32_t *width, uint32_t *height)
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

void FwCam::release_frame()
{
  if (frame_released)
    return; // nothing to do
  dc1394_dma_done_with_buffer(&cam);
  frame_released = true;
}
