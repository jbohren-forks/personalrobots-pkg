#ifndef __PR2MISC_H__
#define __PR2MISC_H__

#include <cmath>
#include <cstdio>
#include <angles/angles.h>

namespace PR2
{
    using namespace angles;
    
    /* --------------- utility functions ---------------- */
    /* ---------------        TODO       ---------------- */
    /* --------------- move to a library ---------------- */

    inline point Rot2D(point p,double theta)
    {
       point q;
       q.x = cos(theta)*p.x - sin(theta)*p.y;
       q.y = sin(theta)*p.x + cos(theta)*p.y;
       return q;
    }

    inline point Rot2D(double x, double y, double theta)
    {
       point q;
       q.x = cos(theta)*x - sin(theta)*y;
       q.y = sin(theta)*x + cos(theta)*y;
       return q;
    }

    inline point Rot2D(point3 p,double theta)
    {
       point q;
       q.x = cos(theta)*p.x - sin(theta)*p.y;
       q.y = sin(theta)*p.x + cos(theta)*p.y;
       return q;
    }

    inline bool IsPTZLeft(PR2_MODEL_ID id)
    {
       if(id == PR2_LEFT_PTZ)
          return true;
       return false;
    }

    inline bool IsPTZRight(PR2_MODEL_ID id)
    {
       if(id == PR2_RIGHT_PTZ)
          return true;
       return false;
    }

    inline bool IsPTZRight(PR2_JOINT_ID id)
    {
       if (id >= JointStart[PR2_RIGHT_PTZ] && id <= JointEnd[PR2_RIGHT_PTZ])
          return true;
       return false;
    }
    inline bool IsPTZLeft(PR2_JOINT_ID id)
    {
       if (id >= JointStart[PR2_LEFT_PTZ] && id <= JointEnd[PR2_LEFT_PTZ])
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

}

#endif
