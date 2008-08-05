#include "keypoint_utils.h"
#include <cstdio>

void WriteKeypointsFl(std::string file_name, std::vector<KeypointFl> const& pts)
{
    FILE* file = fopen(file_name.c_str(), "w");
    if (file) {
        fprintf(file, "%d\n", pts.size());

        typedef std::vector<KeypointFl>::const_iterator iter;
        for (iter i = pts.begin(); i != pts.end(); ++i) {
            fprintf(file, "%f %f %f %f\n", i->x, i->y, i->scale, i->response);
        }
        fclose(file);
    }
}

std::vector<KeypointFl> ReadKeypointsFl(std::string file_name)
{
    std::vector<KeypointFl> pts;
    FILE* file = fopen(file_name.c_str(), "r");
    if (file) {
        int num_pts;
        fscanf(file, "%d\n", &num_pts);
        pts.resize(num_pts);
        for (int i = 0; i < num_pts; ++i) {
            fscanf(file, "%f %f %f %f\n", &pts[i].x, &pts[i].y, &pts[i].scale, &pts[i].response);
        }
    }

    return pts;
}

// Find the overlap error between two circles of radii r1 and r2 with
// distance d between their centers. The overlap error is one minus
// the ratio of the intersection area and the union area.
float OverlapError(float r1, float r2, float d)
{
    // Check if no overlap
    if (d >= r1 + r2) return 1;

    float R = std::max(r1, r2);
    float r = std::min(r1, r2);
    
    float RR = R*R;
    float rr = r*r;
    float dd = d*d;
    float A_R = M_PI * RR;
    float A_r = M_PI * rr;

    // Check if full overlap
    if (d <= R - r) return 1 - A_r/A_R;

    // TODO: explain what's going on here
    
    float d1 = (dd - rr + RR) / (2*d);
    float d2 = d - d1;
    float A1 = CircularSegmentArea(R, d1);
    float A2 = CircularSegmentArea(r, d2); // TODO: what if d2 == 0?
    float A_intersect = A1 + A2;
    if (A_intersect < 0) A_intersect += A_r;
    float A_union = A_R + A_r - A_intersect;

    return 1 - A_intersect/A_union;
}
