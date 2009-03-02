#include <iostream>
#include <sbpl_arm_planner/headers.h>

void getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number)
{
    double delta,rpy1[3],rpy2[3];

     // Check that pitch is not at a singularity
    if(fabs(Rot[0][2]) >= 1)
    {
        rpy1[2]  = 0;
        rpy2[2]  = 0;

        // From difference of angles formula
        delta = atan2(Rot[0][0], Rot[2][0]);
        if(Rot[0][2] > 0)   //gimbal locked up
        {
            rpy1[1] = PI_CONST / 2.0;
            rpy2[1] = PI_CONST / 2.0;
            rpy1[0] = rpy1[1] + delta;
            rpy2[0] = rpy2[1] + delta;
        }
        else // gimbal locked down
        {
            rpy1[1] = -PI_CONST / 2.0;
            rpy2[1] = -PI_CONST / 2.0;
            rpy1[0] = -rpy1[1] + delta;
            rpy2[0] = -rpy2[1] + delta;
	}
    }
    else
    {
        rpy1[1] = -asin(Rot[0][2]);
	rpy2[1] = PI_CONST - rpy1[1];


        rpy1[0] = atan2(Rot[1][2]/cos(rpy1[1]),
                     Rot[2][2]/cos(rpy1[1]));

        rpy2[0] = atan2(Rot[1][2]/cos(rpy2[1]),
                     Rot[2][2]/cos(rpy2[1]));

        rpy1[2] = atan2(Rot[0][1]/cos(rpy1[1]),
                     Rot[0][0]/cos(rpy1[1]));

        rpy2[2] = atan2(Rot[0][1]/cos(rpy2[1]),
                     Rot[0][0]/cos(rpy2[1]));
    }

    if (solution_number == 1)
    {
        *yaw = rpy1[2];
        *pitch = rpy1[1];
        *roll = rpy1[0];
    }
    else
    {
        *yaw = rpy2[2];
        *pitch = rpy2[1];
        *roll = rpy2[0];
    }
}

int main(int argc, char *argv[])
{
    double Rot[3][3] = {{0, -1, 0},
                        {1, 0, 0},
                        {0,0,1}};
//                         {{0,1,0},
//                         {-1,0,0},
//                         {0,0,1}};
    double roll = 0, pitch = 0, yaw = 0;

    printf("Rotation Matrix: \n");
    for(int x=0; x < 3; x++)
    {
        for(int y=0; y < 3; y++)
        {
            printf("%1.3f ",Rot[x][y]);
        }
        printf("\n");
    }
    printf("\n");

    getRPY(Rot, &roll, &pitch, &yaw, 1);
    printf("sol1--> roll: %1.3f  pitch: %1.3f  yaw: %1.3f\n", roll,pitch,yaw);

    getRPY(Rot, &roll, &pitch, &yaw, 2);
    printf("sol2--> roll: %1.3f  pitch: %1.3f  yaw: %1.3f\n", roll,pitch,yaw);
    return 0;
}

