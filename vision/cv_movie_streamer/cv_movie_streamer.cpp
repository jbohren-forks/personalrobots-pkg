#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "simple_options/simple_options.h"

using namespace std;
using namespace ros;

class CvMovieStreamer : public node
{
public:
  string movie_fname;
  int delay, loop;
  CvMovieStreamer(const string &_movie_fname, int _delay, int _loop) 
  : node("cv_movie_streamer"), movie_fname(_movie_fname), 
    delay(_delay), loop(_loop)
  { 
  }
  void stream_movie()
  {
    printf("press any key in the movie window to stop.\n");
    cvNamedWindow("movie stream", CV_WINDOW_AUTOSIZE);
    IplImage *img = NULL;
    int frames = 0;
    do
    {
      CvCapture *capture = cvCaptureFromAVI(movie_fname.c_str());
      if (!capture)
        log(FATAL, "woah! couldn't open video file [%s]",
            movie_fname.c_str());
      while ((img = cvQueryFrame(capture)))
      {
        frames++;
        cvShowImage("movie stream", img);
        if (cvWaitKey(delay) >= 0)
        {
          loop = 0;
          break;
        }
      }
      if (frames == 0)
        loop = 0; // don't loop if we can't even play it once. come on.
      cvReleaseCapture(&capture);
    } while (loop);
    printf("sent %d frames\n", frames);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc <= 1)
  {
    printf("\nusage: cv_movie_streamer MOVIEFILE [OPTIONS]\n\n");
    printf("options:\n");
    printf("  frame_delay_ms=123   : delay inserted between frame decodes\n");
    printf("  loop=1               : loop playback of movie file\n\n");
    return 1;
  }
  map<string,string> opts = simple_options::parse(argc, argv, 2);
  int delay = 5, loop = 0;
  for (map<string,string>::iterator i = opts.begin(); i != opts.end(); ++i)
  {
    string key = (*i).first, value = (*i).second;
    if (key == string("frame_delay_ms"))
      delay = atoi(value.c_str());
    else if (key == string("loop"))
      loop  = atoi(value.c_str());
  }
  printf("movie file: [%s]\n", argv[1]);
  printf("frame_delay_ms = %d\n", delay);
  printf("loop = %d\n", loop);
  CvMovieStreamer ms(argv[1], delay, loop);
  ms.stream_movie();
  ros::fini();
  return 0;
}

