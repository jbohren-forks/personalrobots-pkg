


#define FROM_DEGREES(degrees)	(((double)(degrees))/180*M_PI)
/*
 * normalize_angle_positive
 *
 * Normalizes the angle to be 0 circle to 1 circle
 * It takes and returns native units.
 */

double normalize_angle_positive(double angle)
{
  return fmod(fmod(angle, FROM_DEGREES(360))+FROM_DEGREES(360), FROM_DEGREES(360));
}
/*
 * normalize
 *
 * Normalizes the angle to be -1/2 circle to +1/2 circle
 * It takes and returns native units.
 *
 */

double normalize_angle(double angle)
{
  double a=normalize_angle_positive(angle);
  if (a>FROM_DEGREES(180))
      a-=FROM_DEGREES(360);
  return(a);
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

double shortest_angular_distance(double from, double to)
{
  double result;
  result=normalize_angle_positive(
      normalize_angle_positive(to) - normalize_angle_positive(from));

  if ( result > FROM_DEGREES(180) ) {  // If the result > 180,
                                       // It's shorter the other way.
      result=-(FROM_DEGREES(360)-result);   
  }
  return normalize_angle(result);
}



