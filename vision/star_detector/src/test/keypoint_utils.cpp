#include "keypoint_utils.h"
#include <cstdio>

void WriteKeypointsFl(std::string file_name, std::vector<KeypointFl> const& pts)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (file) {
    fprintf(file, "# %d points\n", pts.size());
    
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
    fscanf(file, "# %d points\n", &num_pts);
    pts.resize(num_pts);
    for (int i = 0; i < num_pts; ++i) {
      fscanf(file, "%f %f %f %f\n", &pts[i].x, &pts[i].y, &pts[i].scale, &pts[i].response);
    }
  }
  
  return pts;
}

//! Find the overlap error between two circles of radii r1 and r2 with
//! distance d between their centers. The overlap error is one minus
//! the ratio of the intersection area and the union area.
float OverlapError(float r1, float r2, float d)
{
    // If circles do not overlap, 100% overlap error
    if (d >= r1 + r2) return 1;

    // Rename to larger radius (R) and smaller radius (r)
    float R = std::max(r1, r2);
    float r = std::min(r1, r2);
    
    float RR = R*R;
    float rr = r*r;
    float dd = d*d;
    float A_R = M_PI * RR;
    float A_r = M_PI * rr;

    // Trivial case: smaller circle completely within the larger circle
    if (d <= R - r) return 1 - A_r/A_R;

    // There are two remaining cases, depending on which side of the common
    // chord between the two intersection points of the circles the center
    // of the smaller circle lies.
    // (1) The centers are on opposite sides of the chord. Then the intersection
    //     area is an asymmetrical lens shape. This case is addressed at
    //     http://mathworld.wolfram.com/Circle-CirlceIntersection.html
    // (2) The centers are on the same side of the chord. Then the intersection
    //     area is the area of the smaller circle minus a circular cusp
    //     protruding from the larger circle.

    // Calculate distances from circle centers to the common chord. They sum to d.
    float d1 = (dd - rr + RR) / (2*d);
    float d2 = d - d1; // d2 is negative in case (2)
    /*
    // Find area of the circular segment defined by the chord for both circles.
    float A1 = CircularSegmentArea(R, d1);
    float A2 = CircularSegmentArea(r, d2); // negative area in case (2)
    float A_intersect = A1 + A2;
    
    if (A_intersect < 0) {
        // Here A_intersect is the negative area of the protruding circular cusp.
        // We add the area of the smaller circle to get the true intersection area.
        A_intersect += A_r;
    }
    */
    float A1 = CircularSegmentArea(R, d1);
    float A2 = CircularSegmentArea(r, fabs(d2));
    float A_intersect;
    if (d2 > 0) {
      A_intersect = A1 + A2;
    } else {
      A_intersect = A_r - A2 + A1;
    }
    float A_union = A_R + A_r - A_intersect;

    return 1 - A_intersect/A_union;
}
