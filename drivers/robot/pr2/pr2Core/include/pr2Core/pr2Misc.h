#ifndef __PR2MISC_H__
#define __PR2MISC_H__

#include<math.h>
#include<stdio.h>

namespace PR2
{

    /* --------------- utility functions ---------------- */
    /* ---------------        TODO       ---------------- */
    /* --------------- move to a library ---------------- */

    inline point Rot2D(point p,double theta)
    {
       point q;
       q.x = cos(theta)*p.x - sin(theta)*p.y;
       q.y = sin(theta)*p.y + cos(theta)*p.x;
       return q;
    }

    inline point Rot2D(point3 p,double theta)
    {
       point q;
       q.x = cos(theta)*p.x - sin(theta)*p.y;
       q.y = sin(theta)*p.y + cos(theta)*p.x;
       return q;
    }

    inline bool IsHead(PR2_MODEL_ID id)
    {
       if(id == HEAD)
          return true;
       return false;
    }
    inline bool IsGripperLeft(PR2_MODEL_ID id)
    {
       if(id == PR2_LEFT_GRIPPER)
          return true;
       return false;
    }
    inline bool IsGripperRight(PR2_MODEL_ID id)
    {
       if(id == PR2_RIGHT_GRIPPER)
          return true;
       return false;
    }

    inline bool IsHead(PR2_JOINT_ID id)
    {
       if(id >= JointStart[HEAD] && id <= JointEnd[HEAD])
          return true;
       return false;
    }

    inline bool IsGripperLeft(PR2_JOINT_ID id)
    {
       if (id >= JointStart[PR2_LEFT_GRIPPER] && id <= JointEnd[PR2_LEFT_GRIPPER])
          return true;
       return false;
    }
    inline bool IsGripperRight(PR2_JOINT_ID id)
    {
       if (id >= JointStart[PR2_RIGHT_GRIPPER] && id <= JointEnd[PR2_RIGHT_GRIPPER])
          return true;
       return false;
    }

    inline double GetMagnitude(double xl[], int num)
    {
       int ii;
       double mag=0;
       for(ii=0; ii < num; ii++)
          mag += (xl[ii]*xl[ii]); 
       return sqrt(mag);
    }

    inline double GetMagnitude(double x, double y)
    {
       return sqrt(x*x+y*y);
    }


    #define FROM_DEGREES(degrees)	(((double)(degrees))/180*M_PI)
    /*
     * normalize_angle_positive
     *
     * Normalizes the angle to be 0 circle to 1 circle
     * It takes and returns native units.
     */

    inline double normalize_angle_positive(double angle)
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

    inline double normalize_angle(double angle)
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

    inline double shortest_angular_distance(double from, double to)
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



}

#endif
