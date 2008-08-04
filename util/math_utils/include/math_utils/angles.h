#ifndef MATH_UTILS_ANGLES
#define MATH_UTILS_ANGLES

#include <cmath>

/** Convert degrees to radians */

static inline double from_degrees(double degrees)
{
    return degrees * M_PI / 180.0;   
}

/** Convert radians to degrees */
static inline double to_degrees(double radians)
{
    return radians * 180.0 / M_PI;
}

/*
 * normalize_angle_positive
 *
 * Normalizes the angle to be 0 circle to 1 circle
 * It takes and returns native units.
 */

static inline double normalize_angle_positive(double angle)
{
    return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}
/*
 * normalize
 *
 * Normalizes the angle to be -1/2 circle to +1/2 circle
 * It takes and returns native units.
 *
 */

static inline double normalize_angle(double angle)
{
  double a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 *M_PI;
  return a;
}

/*
 * shortest_angular_distance
 *
 * Given 2 angles, this returns the shortest angular
 * difference.  The inputs and ouputs are of course native
 * units.
 *
 * As an example, if native units are degrees, the result
 * would always be -180 <= result <= 180.  Adding the result
 * to "from" will always get you an equivelent angle to "to".
 */

static inline double shortest_angular_distance(double from, double to)
{
    double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));

    if (result > M_PI)
	// If the result > 180,
	// It's shorter the other way.
	result = -(2.0*M_PI - result);
    
    return normalize_angle(result);
}


#endif
