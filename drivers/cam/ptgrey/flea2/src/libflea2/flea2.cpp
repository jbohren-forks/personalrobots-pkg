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

Flea2::Flea2(int n_host) :
  raw_created(false), cam_created(false)
{
  raw = dc1394_create_handle(n_host);
  if (!raw)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle\n\n"
                     "Please check \n"
            	       "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded\n"
            	       "  - if you have read/write access to /dev/raw1394\n" );
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
  if(nodes[0] == num_nodes-1)
  {
    fprintf(stderr, "the camera is the root node. that's bad.\n");
    throw std::runtime_error("woah");
  }

  unsigned int channel, speed;
  if (dc1394_get_iso_channel_and_speed(raw, nodes[0], &channel, &speed)
      != DC1394_SUCCESS)
  {
    fprintf(stderr, "Unable to get the iso channel number\n" );
    throw std::runtime_error("toast");
  }
  char host_dev[200];
  snprintf(host_dev, sizeof(host_dev), "/dev/video1394/%d", n_host);
  int e = dc1394_dma_setup_capture(raw, nodes[0], channel, 
    FORMAT_VGA_NONCOMPRESSED, MODE_640x480_RGB, SPEED_400,
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

bool Flea2::get_jpeg(const uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size)
{
  if (dc1394_dma_single_capture(&cam) != DC1394_SUCCESS) 
  {
    printf("couldn't get an image from the camera.\n");
    return false;
  }
  *fetch_buf_size = jpeg.compress_to_jpeg((uint8_t *)cam.capture_buffer, 
                                        cam.frame_width, 
                                        cam.frame_height);
  *fetch_jpeg_buf = jpeg.get_compress_buf();
  dc1394_dma_done_with_buffer(&cam);
  return true;
}
#if 0
int main(int argc, char *argv[]) 
{
   /*-----------------------------------------------------------------------
    *  have the camera start sending us data
    *-----------------------------------------------------------------------*/
   }

   printf( "Capturing 30 images for profiling\n" );

   struct timeval start, end;
   gettimeofday( &start, NULL );

   for ( int i = 0; i < 30; i++ )
   {
     printf( "Capturing image %d...\n", i );
     if ( dc1394_dma_single_capture( &dc1394Camera ) != DC1394_SUCCESS ) 
     {
       printf( "capture %d failed\n", i );
     }
     else 
     {
       save_image(i);
       dc1394_dma_done_with_buffer( &dc1394Camera );
     }
   }

   gettimeofday( &end, NULL );
   int endms = end.tv_sec*1000 + end.tv_usec/1000;
   int startms = start.tv_sec*1000 + start.tv_usec/1000;
   int ms = endms - startms;
   double fps = 30.0*1000.0/(double)ms;
   
   printf( "Grabbed 30 frames in %d milliseconds or %f fps\n", ms, fps );

   
   /*-----------------------------------------------------------------------
    *  capture one more frame to save to file
    *-----------------------------------------------------------------------*/
   printf( "Capturing one image to save to file...\n" );
   if (dc1394_dma_single_capture( &dc1394Camera )!=DC1394_SUCCESS) 
   {
      fprintf( stderr, "Unable to capture a frame\n");
      cleanup();
      return 1;
   }
   
   /*-----------------------------------------------------------------------
    *  save image 
    *-----------------------------------------------------------------------*/
  /* 
   if( imagefile == NULL)
   {
      perror("");
      fprintf( stderr, "Can't create '%s'\n", saveFileName );
      cleanup();
      return 1;
   }
   
   fprintf( imagefile,
	    "P6\n%u %u\n255\n", 
	    dc1394Camera.frame_width,
	    dc1394Camera.frame_height );
   if ( bytesPerPixel == 1 )
   {
      fwrite( (const char *)dc1394Camera.capture_buffer, 1,
	      dc1394Camera.frame_height*dc1394Camera.frame_width, imagefile );
   }
   else
   {
      // write only the first byte of each pixel
      unsigned char* p = (unsigned char*) dc1394Camera.capture_buffer;
      for ( int i = 0; i < dc1394Camera.frame_height*dc1394Camera.frame_width; i++ )
      {
	 fwrite( p, 3, 1, imagefile );
	 p += bytesPerPixel;
      }
   }
   fclose( imagefile );
  */
   //printf( "saved image to '%s'\n", saveFileName );

   dc1394_dma_done_with_buffer( &dc1394Camera );

   /*-----------------------------------------------------------------------
    *  Stop data transmission
    *-----------------------------------------------------------------------*/
   if ( dc1394_stop_iso_transmission( raw1394Handle,dc1394Camera.node ) != DC1394_SUCCESS ) 
   {
      printf("Couldn't stop the camera?\n");
   }
   
   
   /*-----------------------------------------------------------------------
    *  Close camera
    *-----------------------------------------------------------------------*/
   cleanup();
   return 0;
}
#endif

/*************************************************************************
 *
 * $RCSfile: grabdma.cpp,v $
 * $Revision: 1.4 $
 * $Date: 2005/04/07 21:05:14 $
 * $Log: grabdma.cpp,v $
 * Revision 1.4  2005/04/07 21:05:14  donm
 * [1] fixed a bug in grabdma - it was using FRAMERATE_120 which was wrong
 * [2] added a user-configuration section that includes specifying which
 *     interface card the user is using
 *
 * Revision 1.3  2005/03/15 19:32:41  donm
 * updating the LGPL notice etc
 *
 *
 *************************************************************************/
