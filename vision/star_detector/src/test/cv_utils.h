#ifndef FEATURES_TEST_COMMON_UTILS_H
#define FEATURES_TEST_COMMON_UTILS_H

#include "keypoint_utils.h"
#include <cv.h>
#include <highgui.h>

void DrawPoints(IplImage* dst, std::vector<Keypoint> const& pts,
                CvScalar color = CV_RGB(0,255,0), float scale_multiplier = 1.5)
{
    typedef std::vector<Keypoint>::const_iterator iter;

    for (iter i = pts.begin(); i != pts.end(); ++i) {
        int r = scale_multiplier * i->scale;

        cvCircle(dst, cvPoint(i->x, i->y), r, color, 1);
    }
}

struct OnMouseData
{
    std::vector<Keypoint>* keypts;
    float multiplier;
    std::vector<KeypointFl>* keypts_fl;

    OnMouseData(std::vector<Keypoint> &pts, float mult = 1.5,
                std::vector<KeypointFl>* pts_fl = NULL)
        : keypts(&pts), multiplier(mult), keypts_fl(pts_fl)
    {};
};

void on_mouse_find_keypts(int event, int x, int y, int flags, void* param)
{
    typedef std::vector<Keypoint>::const_iterator iter;
    
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            OnMouseData* data = (OnMouseData*) param;
            std::vector<Keypoint> *keypts = data->keypts;
            std::vector<KeypointFl> *keypts_fl = data->keypts_fl;
            
            for (iter i = keypts->begin(); i != keypts->end(); ++i) {
                float r = data->multiplier * i->scale;

                if (fabs(i->x - x) + fabs(i->y - y) <= r) {
                    if (keypts_fl) {
                        KeypointFl &pt = (*keypts_fl)[i - keypts->begin()];
                        printf("Keypoint: (%f, %f, %f), response = %f, line response = %f\n",
                               pt.x, pt.y, pt.scale, pt.response, pt.line_response);
                    } else {
                        printf("Keypoint: (%d, %d, %f), response = %f, line response = %f\n",
                               i->x, i->y, i->scale, i->response, i->line_response);
                    }
                    break;
                }
            }

            break;
    }
}

#endif
