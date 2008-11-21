#include <libKinematics/kinematics.h>
#include <math.h>
#include <libKinematics/pr2_ik.h>

#define NUM_JOINTS_ARM7DOF 7
#define IK_EPS 1e-5

using namespace kinematics;
using namespace std;
using namespace NEWMAT;

arm7DOF::arm7DOF(std::vector<NEWMAT::Matrix> anchors, std::vector<NEWMAT::Matrix> axis, std::vector<std::string> joint_type) : SerialRobot(7)
{
  std::vector<double> angles;
  NEWMAT::Matrix p_axis;
/* Joint angles and speeds for testing */
   double angles_d[7] = {0,0,0,0,0,0,0};

  if(anchors.size() != NUM_JOINTS_ARM7DOF)
  {
    std::cout << "Input joint anchors are of size" << anchors.size() << ", should be 7." << std::endl;
    return;
  }
  if(axis.size() != NUM_JOINTS_ARM7DOF)
  {
    std::cout << "Input axes are not of size 7" << std::endl;
    return;
  }
  if(joint_type.size() != NUM_JOINTS_ARM7DOF)
  {
    std::cout << "Input types are not of size 7" << std::endl;
    return;
  }

  angles.resize(NUM_JOINTS_ARM7DOF);
  axis_.resize(NUM_JOINTS_ARM7DOF);
  anchors_.resize(NUM_JOINTS_ARM7DOF);
  xi_0_.resize(NUM_JOINTS_ARM7DOF);
  xi_0_inv_.resize(NUM_JOINTS_ARM7DOF);


  for(int i=0; i < NUM_JOINTS_ARM7DOF; i++)
  {
    AddJoint(anchors[i],axis[i],joint_type[i]);

    axis_[i] = axis[i];
    anchors_[i] = anchors[i];

    angles[i] = 0.0;
  }

  p_axis = GetJointAxisPoint(1);
  ap_[0] = p_axis(1,1);
  p_axis = GetJointAxisPoint(3);
  ap_[1] = p_axis(1,1);
  p_axis = GetJointAxisPoint(4);
  ap_[2] = p_axis(1,1);
  p_axis = GetJointAxisPoint(5);
  ap_[3] = p_axis(1,1);
  p_axis = GetJointAxisPoint(6);
  ap_[4] = p_axis(1,1);

  for(int i=0; i < NUM_JOINTS_ARM7DOF; i++)
  {
    xi_0_[i] = GetJointExponential(i,0);
    xi_0_inv_[i] = matInv(xi_0_[i]);
  }
   NEWMAT::Matrix g0 = GetLinkPose(7,angles_d);
   this->SetHomePosition(g0);
   solution_.resize(NUM_JOINTS_ARM7DOF);

}

NEWMAT::Matrix arm7DOF::matInv(const NEWMAT::Matrix &g)
{
  NEWMAT::Matrix result = g;
  NEWMAT::Matrix p(3,1);
  NEWMAT::Matrix Rt(3,3);

  Rt(1,1) = g(1,1);
  Rt(2,2) = g(2,2);
  Rt(3,3) = g(3,3);

  Rt(1,2) = g(2,1);
  Rt(2,1) = g(1,2);

  Rt(1,3) = g(3,1);
  Rt(3,1) = g(1,3);

  Rt(2,3) = g(3,2);
  Rt(3,2) = g(2,3);

  p(1,1) = g(1,4);
  p(2,1) = g(2,4);
  p(3,1) = g(3,4);

  NEWMAT::Matrix pinv = -Rt*p;

  result(1,1) = g(1,1);
  result(2,2) = g(2,2);
  result(3,3) = g(3,3);

  result(1,2) = g(2,1);
  result(2,1) = g(1,2);

  result(1,3) = g(3,1);
  result(3,1) = g(1,3);

  result(2,3) = g(3,2);
  result(3,2) = g(2,3);

  result(1,4) = pinv(1,1);
  result(2,4) = pinv(2,1);
  result(3,4) = pinv(3,1);
  
  return result;
}

void arm7DOF::ComputeIK(NEWMAT::Matrix g, double theta1)
{
  solution_ik_.clear();
  /*** A first solution using the shoulder rotation as a parameterization ***/
  /* Find shoulder offset point */
  int jj, kk, ll;

  double t1,t2,t3,t4,t5,t6,t7;

  NEWMAT::Matrix sop(3,1), distance(3,1);

  NEWMAT::Matrix g0 = GetHomePosition();
  NEWMAT::Matrix g0_inv_ = matInv(g0);

  NEWMAT::Matrix g0Wrist = GetHomePosition();

  NEWMAT::Matrix xi1,xi2,xi3,xi4,xi5,xi6,xi7;
  NEWMAT::Matrix p,q,r;
  NEWMAT::Matrix pWrist,qWrist,rWrist;
  NEWMAT::Matrix q0;
  NEWMAT::Matrix xi1_inv_,xi2_inv_,xi3_inv_,xi4_inv_,xi5_inv_,xi6_inv_;

  NEWMAT::Matrix theta2(2,1),theta3(2,1),theta4(2,1),theta5(2,1),theta6(2,1);

  double dd, numerator, denominator, acosTerm;

  double a1x(0), a2x(0), a3x(0), a4x(0), a5x(0);

  NEWMAT::Matrix p_axis;

  p_axis = GetJointAxisPoint(1);
  a1x = p_axis(1,1);
  p_axis = GetJointAxisPoint(3);
  a2x = p_axis(1,1);
  p_axis = GetJointAxisPoint(4);
  a3x = p_axis(1,1);
  p_axis = GetJointAxisPoint(5);
  a4x = p_axis(1,1);
  p_axis = GetJointAxisPoint(6);
  a5x = p_axis(1,1);

  int solutionCount = 1;

//  solution = 0;
  t1 = theta1;

  sop(1,1) = a1x*cos(t1);
  sop(2,1) = a1x*sin(t1);
  sop(3,1) = 0;

  distance = g.SubMatrix(1,3,4,4) - sop;
//  dd = pow(distance.NormFrobenius(),2);
  dd = distance(1,1)*distance(1,1) + distance(2,1)*distance(2,1) + distance(3,1)*distance(3,1);

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
  cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << endl << solution << endl;
  PrintMatrix(theta4,"theta4");/* There are now two solution streams */
#endif

  for(jj =1; jj <= 2; jj++)
  {
    t4 = theta4(jj,1);
      
    if(isnan(t4))
      continue;
    /* Start solving for the other angles */
    q0 = g0.SubMatrix(1,4,4,4);

    xi1 = this->GetJointExponential(0,t1);
    xi4 = this->GetJointExponential(3,t4); 
    xi4_inv_ = matInv(xi4);
    xi1_inv_ = matInv(xi1);

    q = xi1_inv_*g*g0_inv_*xi_0_inv_[6]*xi_0_inv_[5]*xi_0_inv_[4]*q0;
//     q = xi1_inv_*g*g0_inv_*q0;
    p = xi4*q0;
    r = this->GetJointAxisPoint(1);

#ifdef DEBUG
    PrintMatrix(g0,"ComputeIK::g0");
    PrintMatrix(q0,"ComputeIK::q0");
    PrintMatrix(axis_[1],"ComputeIK::omega1");
    PrintMatrix(axis_[2],"ComputeIK::omega2");

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

    double theta_ans[4];
    SubProblem2(p,q,r,axis_[1],axis_[2],theta_ans);
   
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
      if(isnan(t2) || isnan(t3))
        continue;
      /* Two more solutions here, making for a total of 4 solution stream so far */
   
#ifdef DEBUG
      PrintMatrix(solution,"Answer");
#endif

      /* Solve for the wrist angles now. First, take everything but the wrist over to the lhs.
         Use this information to solve for the 5th and 6th axes.
      */
      xi2 = this->GetJointExponential(1,t2);
      xi3 = this->GetJointExponential(2,t3);

      xi2_inv_ = matInv(xi2);
      xi3_inv_ = matInv(xi3);

      rWrist = this->GetJointAxisPoint(6);
      pWrist = rWrist;
      pWrist(1,1) = pWrist(1,1) + 0.1;
      qWrist = xi4_inv_*xi3_inv_*xi2_inv_*xi1_inv_*g*g0_inv_*pWrist;

      SubProblem2(pWrist,qWrist,rWrist,axis_[4],axis_[5],theta_ans);

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
        if(isnan(t5) || isnan(t6))
          continue;
        xi5 = this->GetJointExponential(4,t5);
        xi6 = this->GetJointExponential(5,t6);

        xi5_inv_= matInv(xi5);
        xi6_inv_ = matInv(xi6);

        /* Now use these solutions to solve for the 7th axis */
        pWrist(3,1) = pWrist(3,1) + 0.1;
        qWrist =  xi6_inv_*xi5_inv_*xi4_inv_*xi3_inv_*xi2_inv_*xi1_inv_*g*g0_inv_*pWrist;
        t7 = SubProblem1(pWrist,qWrist,rWrist,axis_[6]);
        if(isnan(t7))
          continue;
        
        solution_[0] = t1;
        solution_[1] = t2;
        solution_[2] = t3;
        solution_[3] = t4;
        solution_[4] = t5;
        solution_[5] = t6;
        solution_[6] = t7;
        solution_ik_.push_back(solution_);
/*
        solution(1,solutionCount) = t1;
        solution(2,solutionCount) = t2;
        solution(3,solutionCount) = t3;
        solution(4,solutionCount) = t4;
        solution(5,solutionCount) = t5;
        solution(6,solutionCount) = t6;
        solution(7,solutionCount) = t7;
*/
//        if (isnan(t1) || isnan(t2) || isnan(t3) || isnan(t4) || isnan(t5) || isnan(t6) || isnan(t7))
//          solution(8,solutionCount) = -1;
#ifdef DEBUG
        cout << "t1: " << t1 << ", t2: " << t2 << ", t3: " << t3 << ", t4: " << t4  << ", t5: " << t5 << ", t6: " << t6 << ", t7: " << t7 << endl;
#endif
        solutionCount++;
      }
    }
  }
//  return solution;
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


int arm7DOF::solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2)
{
   double theta1 = atan2(b,a);
   double denom  = sqrt(a*a+b*b);
   int error_code = 1;

   if(denom < IK_EPS) // should never happen, wouldn't make sense but make sure it is checked nonetheless
   {
      std::cout << "solveCosineEqn: degenerate condition" << endl;
      return -1;
   }
   double rhs_ratio = c/denom;

   if(rhs_ratio < -1)
   {
      rhs_ratio  = -1;
      error_code = 0;
   }
   else if(rhs_ratio > 1)
   {
      rhs_ratio = 1;
      error_code = 0;
   }
   double acos_term = acos(rhs_ratio);
   soln1 = theta1 + acos_term;
   soln2 = theta1 - acos_term;

   return error_code;
}



void arm7DOF::ComputeIKEfficient(NEWMAT::Matrix g, double t1)
{
   int error_code(0);

   double t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);

   double at(0), bt(0), ct(0);

   double theta2[2],theta3[2],theta4[2],theta6[2]; 

   double sopx = ap_[0]*cos(t1);
   double sopy = ap_[0]*sin(t1);
   double sopz = 0;

   double x = g(1,4);
   double y = g(2,4);
   double z = g(3,4);

   double dx = x - sopx;
   double dy = y - sopy;
   double dz = z - sopz;

   double dd = dx*dx + dy*dy + dz*dz;

   double numerator = dd-ap_[0]*ap_[0]+2*ap_[0]*ap_[1]-2*ap_[1]*ap_[1]+2*ap_[1]*ap_[3]-ap_[3]*ap_[3];
   double denominator = 2*(ap_[0]-ap_[1])*(ap_[1]-ap_[3]);

   double acosTerm = numerator/denominator;

   if (acosTerm > 1.0)
      acosTerm = 1.0;
   if (acosTerm < -1.0)
      acosTerm = -1.0;
   
   theta4[0] = acos(acosTerm);
   theta4[1] = -acos(acosTerm);

#ifdef DEBUG
   cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << endl << solution << endl;
   PrintMatrix(theta4,"theta4");/* There are now two solution streams */
#endif

  NEWMAT::Matrix g0 = GetHomePosition();
  NEWMAT::Matrix g0_inv_ = matInv(g0);
  NEWMAT::Matrix grhs = g0;
  NEWMAT::Matrix gf = g*g0_inv_;

   for(int jj =0; jj < 2; jj++)
   {
      t4 = theta4[jj];
      if(isnan(t4))
         continue;

      at = x*cos(t1)+y*sin(t1)-ap_[0];
      bt = -z;
      ct = -ap_[0] + ap_[1] + (ap_[3]-ap_[1])*cos(t4);

      error_code = solveCosineEqn(at,bt,ct,theta2[0],theta2[1]);

      if(error_code != 1)
         continue;

      for(int ii=0; ii < 2; ii++)
      {
         t2 = theta2[ii];

         at = sin(t1)*(ap_[1] - ap_[3])*sin(t2)*sin(t4);
         bt = (-ap_[1]+ap_[3])*cos(t1)*sin(t4);
         ct = y - (ap_[0]+cos(t2)*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cos(t4)))*sin(t1);
         error_code = solveCosineEqn(at,bt,ct,theta3[0],theta3[1]);

         if(error_code != 1)
            continue;

         for(int kk =0; kk < 2; kk++)
         {
            t3 = theta3[kk];

            grhs(1,1) = cos(t4)*(gf(1,1)*cos(t1)*cos(t2)+gf(2,1)*cos(t2)*sin(t1)-gf(3,1)*sin(t2))-(gf(3,1)*cos(t2)*cos(t3) + cos(t3)*(gf(1,1)*cos(t1) + gf(2,1)*sin(t1))*sin(t2) + (-(gf(2,1)*cos(t1)) + gf(1,1)*sin(t1))*sin(t3))*sin(t4);

            grhs(1,2) = cos(t4)*(gf(1,2)*cos(t1)*cos(t2) + gf(2,2)*cos(t2)*sin(t1) - gf(3,2)*sin(t2)) - (gf(3,2)*cos(t2)*cos(t3) + cos(t3)*(gf(1,2)*cos(t1) + gf(2,2)*sin(t1))*sin(t2) + (-(gf(2,2)*cos(t1)) + gf(1,2)*sin(t1))*sin(t3))*sin(t4);

            grhs(1,3) = cos(t4)*(gf(1,3)*cos(t1)*cos(t2) + gf(2,3)*cos(t2)*sin(t1) - gf(3,3)*sin(t2)) - (gf(3,3)*cos(t2)*cos(t3) + cos(t3)*(gf(1,3)*cos(t1) + gf(2,3)*sin(t1))*sin(t2) + (-(gf(2,3)*cos(t1)) + gf(1,3)*sin(t1))*sin(t3))*sin(t4);

            grhs(2,1) = cos(t3)*(gf(2,1)*cos(t1) - gf(1,1)*sin(t1)) + gf(3,1)*cos(t2)*sin(t3) + (gf(1,1)*cos(t1) + gf(2,1)*sin(t1))*sin(t2)*sin(t3);

            grhs(2,2) = cos(t3)*(gf(2,2)*cos(t1) - gf(1,2)*sin(t1)) + gf(3,2)*cos(t2)*sin(t3) + (gf(1,2)*cos(t1) + gf(2,2)*sin(t1))*sin(t2)*sin(t3);

            grhs(2,3) = cos(t3)*(gf(2,3)*cos(t1) - gf(1,3)*sin(t1)) + gf(3,3)*cos(t2)*sin(t3) + (gf(1,3)*cos(t1) + gf(2,3)*sin(t1))*sin(t2)*sin(t3);

            grhs(3,1) = cos(t4)*(gf(3,1)*cos(t2)*cos(t3) + cos(t3)*(gf(1,1)*cos(t1) + gf(2,1)*sin(t1))*sin(t2) + (-(gf(2,1)*cos(t1)) + gf(1,1)*sin(t1))*sin(t3)) + (gf(1,1)*cos(t1)*cos(t2) + gf(2,1)*cos(t2)*sin(t1) - gf(3,1)*sin(t2))*sin(t4);

            grhs(3,2) = cos(t4)*(gf(3,2)*cos(t2)*cos(t3) + cos(t3)*(gf(1,2)*cos(t1) + gf(2,2)*sin(t1))*sin(t2) + (-(gf(2,2)*cos(t1)) + gf(1,2)*sin(t1))*sin(t3)) + (gf(1,2)*cos(t1)*cos(t2) + gf(2,2)*cos(t2)*sin(t1) - gf(3,2)*sin(t2))*sin(t4);

            grhs(3,3) = cos(t4)*(gf(3,3)*cos(t2)*cos(t3) + cos(t3)*(gf(1,3)*cos(t1) + gf(2,3)*sin(t1))*sin(t2) + (-(gf(2,3)*cos(t1)) + gf(1,3)*sin(t1))*sin(t3)) + (gf(1,3)*cos(t1)*cos(t2) + gf(2,3)*cos(t2)*sin(t1) - gf(3,3)*sin(t2))*sin(t4);


            theta6[0] = atan2(sqrt(grhs(1,2)*grhs(1,2)+grhs(1,3)*grhs(1,3)),grhs(1,1));
            theta6[1] = atan2(-sqrt(grhs(1,2)*grhs(1,2)+grhs(1,3)*grhs(1,3)),grhs(1,1));

            for(int mm = 0; mm < 2; mm++)
            {
               t6 = theta6[mm];

               if(fabs(sin(t6)) < IK_EPS)
               {
                  std::cout << "Singularity" << endl;
                  t5 = acos(grhs(2,2))/2.0;
                  t7 = acos(grhs(2,2))/2.0;
               }
               else
               {
                  t7 = atan2(grhs(1,2),grhs(1,3));
                  t5 = atan2(grhs(2,1),-grhs(3,1));
               }
/*               std::cout << "theta1: " << t1 << endl;
               std::cout << "theta2: " << t2 << endl;
               std::cout << "theta3: " << t3 << endl;
               std::cout << "theta4: " << t4 << endl;
               std::cout << "theta5: " << t5 << endl;
               std::cout << "theta6: " << t6 << endl;
               std::cout << "theta7: " << t7 << endl << endl << endl;
*/             solution_[0] = t1;
               solution_[1] = t2;
               solution_[2] = t3;
               solution_[3] = t4;
               solution_[4] = t5;
               solution_[5] = t6;
               solution_[6] = t7;
               solution_ik_.push_back(solution_);

            }
         }
      }
   }
}
