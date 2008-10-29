// This file is temporarily copied over of Hai's scratch project of cloud_transform


#include <stdlib.h>
#include <iostream.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

typedef struct
{
    std::vector<CvPoint3D32f> *points;
    std::vector<double>       *intensity;
} laser_scan;

typedef struct
{
    CvPoint points2d;
    CvPoint3D32f points3d;
} mapped_points;

typedef struct
{
    IplImage *image;
    std::vector<mapped_points> points;
} synthetic_image;

laser_scan read_lrf_dat_file(const char *filename);
synthetic_image image_from_point_cloud(laser_scan &ls, int image_width, int image_height);

