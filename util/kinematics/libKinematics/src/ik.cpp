#include <libKinematics/kinematics.h>
#include <pr2Core/pr2Core.h>
#include <math.h>
#include <libKinematics/ik.h>

double a1x(0.1),a2x(0.5),a3x(0.59085),a4x(0.81455),a5x(0.81455);
using namespace kinematics;
using namespace PR2;
using namespace std;


PR2Arm::PR2Arm()
{
/* Link lengths */
   double p1[3] = {0,0,0};
   double p2[3] = {p1[0]+ARM_PAN_SHOULDER_PITCH_OFFSET.x,p1[1],p1[2]};
   double p3[3] = {p2[0]+ARM_SHOULDER_PITCH_ROLL_OFFSET.x,p2[1],p2[2]};
   double p4[3] = {p3[0]+ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET.x,p3[1],p3[2]};
   double p5[3] = {p4[0]+ELBOW_PITCH_ELBOW_ROLL_OFFSET.x,p4[1],p4[2]};
   double p6[3] = {p5[0]+ELBOW_ROLL_WRIST_PITCH_OFFSET.x,p5[1],p5[2]};
   double p7[3] = {p6[0]+WRIST_PITCH_WRIST_ROLL_OFFSET.x,p6[1],p6[2]};


/* Axis definitions */
   double axis1[3] = {0,0,1};
   double axis2[3] = {0,1,0};
   double axis3[3] = {1,0,0};
   double axis4[3] = {0,1,0};
   double axis5[3] = {1,0,0};
   double axis6[3] = {0,1,0};
   double axis7[3] = {1,0,0};

/* Joint angles and speeds for testing */
   double angles[7] = {0,0,0,0,0,0,0};

   this->Initialize(7);
   this->AddJoint(p1,axis1,kinematics::ROTARY);
   this->AddJoint(p2,axis2,kinematics::ROTARY);
   this->AddJoint(p3,axis3,kinematics::ROTARY);
   this->AddJoint(p4,axis4,kinematics::ROTARY);
   this->AddJoint(p5,axis5,kinematics::ROTARY);
   this->AddJoint(p6,axis6,kinematics::ROTARY);
   this->AddJoint(p7,axis7,kinematics::ROTARY);

   NEWMAT::Matrix g0 = this->GetLinkPose(6,angles);
   this->SetHomePosition(g0);

}

NEWMAT::Matrix PR2Arm::ComputeIK(NEWMAT::Matrix g, double theta1)
{
   /*** A first solution using the shoulder rotation as a parameterization ***/
   /* Find shoulder offset point */
   int jj, kk, ll;

   double t1,t2,t3,t4,t5,t6,t7;

   NEWMAT::Matrix sop(3,1), distance(3,1);
   NEWMAT::Matrix g0 = this->GetHomePosition();
   NEWMAT::Matrix g0Wrist = this->GetHomePosition();
   NEWMAT::Matrix theta(8,8);

   NEWMAT::Matrix xi1,xi2,xi3,xi4,xi5,xi6,xi7;
   NEWMAT::Matrix p,q,r;
   NEWMAT::Matrix omega1,omega2,omega3,omega4,omega5,omega6;
   NEWMAT::Matrix pWrist,qWrist,rWrist;
   NEWMAT::Matrix q0;

   NEWMAT::Matrix theta2(2,1),theta3(2,1),theta4(2,1),theta5(2,1),theta6(2,1);

   double dd, numerator, denominator, acosTerm;

   int solutionCount = 1;

   theta = 0;
   t1 = theta1;

   sop(1,1) = a1x*cos(t1);
   sop(2,1) = a1x*sin(t1);
   sop(3,1) = 0;

   distance = g.SubMatrix(1,3,4,4) - sop;
   dd = pow(distance.NormFrobenius(),2);

#ifdef DEBUG
   PrintMatrix(distance,"ComputeIK::distance::");
   cout << "ComputeIK::dd::" << dd << endl;
   cout << "ComputeIK::a1x::" << a1x << ", a2x::" << a2x << ", a3x::" << a3x << ", a4x::" << a4x << endl;
#endif

   numerator = dd-a1x*a1x+2*a1x*a2x-2*a2x*a2x+2*a2x*a4x-a4x*a4x;
   denominator = 2*(a1x-a2x)*(a2x-a4x);

   acosTerm = numerator/denominator;

   if (acosTerm > 1.0)
      acosTerm = 1.0;
   if (acosTerm < -1.0)
      acosTerm = -1.0;
   
   theta4(1,1) = acos(acosTerm);
   theta4(2,1) = -acos(acosTerm);

#ifdef DEBUG
   cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << endl <<  theta[3] << endl;
   PrintMatrix(theta4,"theta4");/* There are now two solution streams */
#endif

   for(jj =1; jj <= 2; jj++)
   {
      t4 = theta4(jj,1);
      
      /* Start solving for the other angles */
      q0 = g0.SubMatrix(1,4,4,4);
      q0(3,1) = q0(3,1);

      xi1 = this->GetJointExponential(0,t1);
      xi4 = this->GetJointExponential(3,t4); 
      xi5 = this->GetJointExponential(4,0);
      xi6 = this->GetJointExponential(5,0);
      xi7 = this->GetJointExponential(6,0);

      q = xi1.i()*g*g0.i()*xi7.i()*xi6.i()*xi5.i()*q0;
      p = xi4*q0;
      r = this->GetJointAxisPoint(1);

      omega1 = this->GetJointAxis(1);
      omega2 = this->GetJointAxis(2);

#ifdef DEBUG
      PrintMatrix(g0,"ComputeIK::g0");
      PrintMatrix(q0,"ComputeIK::q0");
      PrintMatrix(omega1,"ComputeIK::omega1");
      PrintMatrix(omega2,"ComputeIK::omega2");

      PrintMatrix(g,"ComputeIK::g");
      PrintMatrix(p,"ComputeIK::p");
      PrintMatrix(q,"ComputeIK::q");
      PrintMatrix(r,"ComputeIK::r");

      PrintMatrix(xi1,"ComputeIK::xi1");
      PrintMatrix(xi2,"ComputeIK::xi2");
      PrintMatrix(xi3,"ComputeIK::xi3");
      PrintMatrix(xi4,"ComputeIK::xi4");
      PrintMatrix(xi5,"ComputeIK::xi5");
      PrintMatrix(xi6,"ComputeIK::xi6");
      PrintMatrix(xi7,"ComputeIK::xi7");
#endif

      double *theta_ans = new double[4];
      SubProblem2(p,q,r,omega1,omega2,theta_ans);
   
      theta2(1,1) = theta_ans[0];
      theta2(2,1) = theta_ans[2];

      theta3(1,1) = theta_ans[1];
      theta3(2,1) = theta_ans[3];

#ifdef DEBUG
      PrintMatrix(theta2,"theta2");
      PrintMatrix(theta3,"theta3");
#endif

      for(kk=1; kk <= 2; kk++)
      {
         t2 = theta2(kk,1);
         t3 = theta3(kk,1);
         /* Two more solutions here, making for a total of 4 solution stream so far */
   
#ifdef DEBUG
         PrintMatrix(theta,"Answer");
#endif

         /* Solve for the wrist angles now. First, take everything but the wrist over to the lhs.
            Use this information to solve for the 5th and 6th axes.
         */
         xi2 = this->GetJointExponential(1,t2);
         xi3 = this->GetJointExponential(2,t3);

         rWrist = this->GetJointAxisPoint(6);
         pWrist = rWrist;
         pWrist(1,1) = pWrist(1,1) + 0.1;
         qWrist = xi4.i()*xi3.i()*xi2.i()*xi1.i()*g*g0.i()*pWrist;

         omega4 = this->GetJointAxis(4);
         omega5 = this->GetJointAxis(5);
         omega6 = this->GetJointAxis(6);

         SubProblem2(pWrist,qWrist,rWrist,omega4,omega5,theta_ans);

         /* Two more solutions here, making for a total of 8 solution streams so far */
         theta5(1,1) = theta_ans[0];
         theta6(1,1) = theta_ans[1];
         theta5(2,1) = theta_ans[2];
         theta6(2,1) = theta_ans[3];

#ifdef DEBUG
         PrintMatrix(theta5,"theta5");
         PrintMatrix(theta6,"theta6");
#endif

         for (ll=1; ll <=2; ll++)
         {
            t5 = theta5(ll,1);
            t6 = theta6(ll,1);

            xi5 = this->GetJointExponential(4,t5);
            xi6 = this->GetJointExponential(5,t6);
            /* Now use these solutions to solve for the 7th axis */
            pWrist(3,1) = pWrist(3,1) + 0.1;
            qWrist =  xi6.i()*xi5.i()*xi4.i()*xi3.i()*xi2.i()*xi1.i()*g*g0.i()*pWrist;
            t7 = SubProblem1(pWrist,qWrist,rWrist,omega6);

            theta(1,solutionCount) = t1;
            theta(2,solutionCount) = t2;
            theta(3,solutionCount) = t3;
            theta(4,solutionCount) = t4;
            theta(5,solutionCount) = t5;
            theta(6,solutionCount) = t6;
            theta(7,solutionCount) = t7;

            if (isnan(t1) || isnan(t2) || isnan(t3) || isnan(t4) || isnan(t5) || isnan(t6) || isnan(t7))
               theta(8,solutionCount) = -1;
#ifdef DEBUG
            cout << "t1: " << t1 << ", t2: " << t2 << ", t3: " << t3 << ", t4: " << t4  << ", t5: " << t5 << ", t6: " << t6 << ", t7: " << t7 << endl;
#endif
            solutionCount++;
         }
      }
   }
   return theta;
}


PR2Head::PR2Head()
{
/* Link lengths */
   double p1[3] = {0,0,0};
   double p2[3] = {p1[0]+HEAD_PAN_HEAD_PITCH_OFFSET.x,p1[1]+HEAD_PAN_HEAD_PITCH_OFFSET.y,p1[2]+HEAD_PAN_HEAD_PITCH_OFFSET.z};

/* Axis definitions */
   double axis1[3] = {0,0,1};
   double axis2[3] = {0,1,0};

/* Joint angles and speeds for testing */
   double angles[2] = {0,0};

   this->Initialize(2);
   this->AddJoint(p1,axis1,kinematics::ROTARY);
   this->AddJoint(p2,axis2,kinematics::ROTARY);

   NEWMAT::Matrix g0 = this->GetLinkPose(2,angles);
   this->SetHomePosition(g0);
}



/**** List of angles (for reference) *******
th1 = shoulder/turret pan
th2 = shoulder/turret lift/pitch
th3 = shoulder/turret roll
th4 = elbow pitch
th5 = elbow roll 
th6 = wrist pitch
th7 = wrist roll 
****/
