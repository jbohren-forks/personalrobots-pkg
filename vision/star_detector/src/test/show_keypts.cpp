#include "cv_utils.h"
#include "keypoint_utils.h"
#include <cstdlib>
#include <cassert>
#include <iostream>

char wndname[] = "Image";

int main( int argc, char** argv )
{
    if (argc <= 2) {
        std::cerr << "Usage: ./show_keypts image.pgm pts.key [scale_multiplier]\n";
        return 0;
    }
    
    IplImage* loaded = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    assert(loaded);
    int W = loaded->width;
    int H = loaded->height;

    std::vector<KeypointFl> keypts_fl = ReadKeypointsFl(argv[2]);
    std::vector<Keypoint> keypts;
    typedef std::vector<KeypointFl>::const_iterator iter;
    for (iter i = keypts_fl.begin(); i != keypts_fl.end(); ++i) {
        keypts.push_back(Keypoint(i->x + 0.5, i->y + 0.5, i->scale + 0.5, i->response));
    }

    float scale_multiplier = 1.5;
    if (argc > 3)
        scale_multiplier = atof(argv[3]);
    
    IplImage* loaded_color = cvCreateImage(cvSize(W,H), IPL_DEPTH_8U, 3);
    cvCvtColor(loaded, loaded_color, CV_GRAY2BGR);
    DrawPoints(loaded_color, keypts, CV_RGB(0,255,0), scale_multiplier);

    cvNamedWindow(wndname);
    OnMouseData mouse_data(keypts, scale_multiplier, &keypts_fl);
    cvSetMouseCallback(wndname, on_mouse_find_keypts, &mouse_data);
    cvShowImage(wndname, loaded_color);

    int c = cvWaitKey(0);
    if ((char) c == 's') {
      cvSaveImage("points.ppm", loaded_color);
      std::cout << "Saved image to points.ppm" << std::endl;
    }

    cvReleaseImage(&loaded);
    cvReleaseImage(&loaded_color);
    cvDestroyWindow(wndname);

    return 0;
}
