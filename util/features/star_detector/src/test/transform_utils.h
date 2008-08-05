#ifndef FEATURES_TEST_TRANSFORM_UTILS_H
#define FEATURES_TEST_TRANSFORM_UTILS_H

#include <cv.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string.h> // strtok
#include "keypoint.h"

// Integer pixel version, rounds result
CvPoint MapPoint(CvPoint pt, CvMat* map_matrix)
{
    float* row1 = map_matrix->data.fl;
    float* row2 = row1 + map_matrix->step / sizeof(float);

    float x = pt.x*row1[0] + pt.y*row1[1] + row1[2];
    float y = pt.x*row2[0] + pt.y*row2[1] + row2[2];

    // Handle general homography
    float w = 1;
    if (map_matrix->rows > 2) {
        float* row3 = row2 + map_matrix->step / sizeof(float);
        w = pt.x*row3[0] + pt.y*row3[1] + row3[2];
    }
    
    return cvPoint(x/w + 0.5, y/w + 0.5);
}

// FP pixel version, no rounding
CvPoint2D32f MapPoint(CvPoint2D32f pt, CvMat* map_matrix)
{
    float* row1 = map_matrix->data.fl;
    float* row2 = row1 + map_matrix->step / sizeof(float);

    float x = pt.x*row1[0] + pt.y*row1[1] + row1[2];
    float y = pt.x*row2[0] + pt.y*row2[1] + row2[2];

    // Handle general homography
    float w = 1;
    if (map_matrix->rows > 2) {
        float* row3 = row2 + map_matrix->step / sizeof(float);
        w = pt.x*row3[0] + pt.y*row3[1] + row3[2];
    }
    
    return cvPoint2D32f(x/w, y/w);
}

// Functor to reject points that project back to somewhere outside
// or on the border of the source image.
struct OutsideSource
{
    int W, H;
    CvMat* inv_matrix;

    OutsideSource(int width, int height, CvMat* inv)
        : W(width), H(height), inv_matrix(inv)
    {}

    template< typename KeyptT >
    inline bool operator() (KeyptT const& pt) {
        CvPoint2D32f back = MapPoint(cvPoint2D32f(pt.x, pt.y), inv_matrix);

        float offset = 3*(pt.scale + 1);
        
        return back.x < offset || back.x >= W - offset ||
            back.y < offset || back.y >= H - offset;
    }
};

CvSize WarpedSize(int W, int H, CvMat* map_matrix)
{
    CvPoint tl = MapPoint(cvPoint(0,0), map_matrix);
    CvPoint tr = MapPoint(cvPoint(W-1,0), map_matrix);
    CvPoint bl = MapPoint(cvPoint(0,H-1), map_matrix);
    CvPoint br = MapPoint(cvPoint(W-1,H-1), map_matrix);

    int min_x = MIN(tl.x, MIN(tr.x, MIN(bl.x, br.x)));
    int min_y = MIN(tl.y, MIN(tr.y, MIN(bl.y, br.y)));
    int max_x = MAX(tl.x, MAX(tr.x, MAX(bl.x, br.x)));
    int max_y = MAX(tl.y, MAX(tr.y, MAX(bl.y, br.y)));

    return cvSize(max_x - min_x + 1, max_y - min_y + 1);
}

CvSize FullImageRotation(int W, int H, double angle, CvMat* map_matrix)
{
    // Rotation matrix about center of source image
    cv2DRotationMatrix(cvPoint2D32f(W/2, H/2), angle, 1, map_matrix);

    // Adjust the shift to map to the new warped image center
    CvSize warped_size = WarpedSize(W, H, map_matrix);
    float* map_data = map_matrix->data.fl;
    map_data[2] += warped_size.width/2 - W/2;
    (map_data + map_matrix->step / sizeof(float))[2] += warped_size.height/2 - H/2;

    return warped_size;
}

void InvertRotation(CvMat* map_matrix, CvMat* inv_matrix)
{
    float* map_data = map_matrix->data.fl;
    float* inv_data = inv_matrix->data.fl;
    float alpha = map_data[0], beta = map_data[1], dx = map_data[2];
    float dy = (map_data + map_matrix->step / sizeof(float))[2];
    inv_data[0] = alpha;
    inv_data[1] = -beta;
    inv_data[2] = -alpha*dx + beta*dy;
    inv_data += inv_matrix->step / sizeof(float);
    inv_data[0] = beta;
    inv_data[1] = alpha;
    inv_data[2] = -beta*dx - alpha*dy;
}

void WriteTransform(std::string file_name, CvMat* transform)
{
    FILE* file = fopen(file_name.c_str(), "w");
    if (file) {
        float* data = transform->data.fl;
        fprintf(file, "%f %f %f\n", data[0], data[1], data[2]);
        data += transform->step / sizeof(float);
        fprintf(file, "%f %f %f\n", data[0], data[1], data[2]);
        if (transform->rows > 2) {
            data += transform->step / sizeof(float);
            fprintf(file, "%f %f %f\n", data[0], data[1], data[2]);
        } else {
            fprintf(file, "0 0 1\n");
        }
        fclose(file);
    }
}

bool ReadTriple(FILE* file, float* data)
{
    const char sep[] = " \t\n";
    char line[128];
    if (! fgets(line, 128, file)) return false;

    char* token = strtok(line, sep);
    data[0] = atof(token);
    token = strtok(NULL, sep);
    data[1] = atof(token);
    token = strtok(NULL, sep);
    data[2] = atof(token);

    return true;
}

void ReadTransform(std::string file_name, CvMat* transform)
{
    FILE* file = fopen(file_name.c_str(), "r");
    if (file) {
        float* data = transform->data.fl;
        ReadTriple(file, data);
        data += transform->step / sizeof(float);
        ReadTriple(file, data);
        if (transform->rows > 2) {
            data += transform->step / sizeof(float);
            if (! ReadTriple(file, data)) {
                data[0] = data[1] = 0;
                data[2] = 1;
            }
        }
        fclose(file);
    }
}

#endif
