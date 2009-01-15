//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.


#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>
#include <angles/angles.h>
#include <math.h>

using namespace kinematics;
using namespace std;
using namespace NEWMAT;
using namespace angles;

arm7DOF::arm7DOF(std::vector<NEWMAT::Matrix> anchors, std::vector<NEWMAT::Matrix> axis, std::vector<std::string> joint_type) : SerialRobot(7),increment_(0.01)
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

  angle_multipliers_.resize(NUM_JOINTS_ARM7DOF);

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
    angle_multipliers_[i] = axis_[i](1,1)*fabs(axis_[i](1,1)) +  axis_[i](2,1)*fabs(axis_[i](2,1)) +  axis_[i](3,1)*fabs(axis_[i](3,1));
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

  a1_ = ap_[0];
  a2_ = ap_[1]-ap_[0];
  a4_ = ap_[4]-ap_[1];

  //  cout << "a1: " << a1_ << endl;
  //  cout << "a2: " << a2_ << endl;
  //  cout << "a4: " << a4_ << endl;

  for(int i=0; i < NUM_JOINTS_ARM7DOF; i++)
  {
    xi_0_[i] = GetJointExponential(i,0);
    xi_0_inv_[i] = matInv(xi_0_[i]);
  }

  NEWMAT::Matrix g0 = GetLinkPose(7,angles_d);
  this->SetHomePosition(g0);
  solution_.resize(NUM_JOINTS_ARM7DOF);

  home_inv_ = matInv(g0);
  grhs_ = g0;
  gf_ = home_inv_;

}

bool arm7DOF::setAngleMultipliers(std::vector<double> angle_mult)
{
  if(angle_mult.size() != angle_multipliers_.size())
  {
    printf("Size of input vector %d does not match number of joints %d",(int) angle_mult.size(),(int) angle_multipliers_.size());
    return false;
  }
  cout << "Angle multipliers " ;
  for(int i=0; i< (int) angle_multipliers_.size(); i++)
  {
    angle_multipliers_[i] = angle_mult[i];
    printf("%f ",angle_multipliers_[i]);
  }
  printf("\n");
  return true;
}

int arm7DOF::solve_quadratic(double a, double b, double c, double *x1, double *x2)
{
  double discriminant = b*b-4*a*c;
  if(fabs(a) < IK_EPS)
  {
    *x1 = -c/b;
    *x2 = *x1;
    return 0;
  }

  if (discriminant >= 0)
  {      
    *x1 = (-b + sqrt(discriminant))/(2*a); 
    *x2 = (-b - sqrt(discriminant))/(2*a);
    return 0;
  } 
  else
  {
    *x1 = -b/(2*a);
    *x2 = -b/(2*a);
    return -1;
  }
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

  if(fabs(denom) < IK_EPS) // should never happen, wouldn't make sense but make sure it is checked nonetheless
  {
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
  solution_ik_.clear();

/*   for(int i=0; i < 4; i++)
     {
     for(int j=0; j<4; j++)
     {
     wprintf(L" %f ",g(i+1,j+1));
     }
     wprintf(L"\n");
     }
*/
  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost1 = cos(t1);
  sint1 = sin(t1);

  t1 = angles::normalize_angle(t1);

  int error_code(0);

  double t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);

  double at(0), bt(0), ct(0);

  double theta2[2],theta3[2],theta4[2],theta5[2],theta6[4],theta7[2]; 

  double sopx = ap_[0]*cost1;
  double sopy = ap_[0]*sint1;
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

  if (acosTerm > 1.0 || acosTerm < -1.0)
    return;

  double acos_angle = acos(acosTerm);
 
  theta4[0] = acos_angle;
  theta4[1] = -acos_angle;

#ifdef DEBUG
  cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << endl << theta4[0] << endl;
  PrintMatrix(theta4,"theta4");/* There are now two solution streams */
#endif


  for(int jj =0; jj < 2; jj++)
  {
    t4 = theta4[jj];
    cost4 = cos(t4);
    sint4 = sin(t4);

#ifdef DEBUG
    cout << "t4 " << t4 << endl;
#endif
    if(isnan(t4))
      continue;
    at = x*cost1+y*sint1-ap_[0];
    bt = -z;
    ct = -ap_[0] + ap_[1] + (ap_[3]-ap_[1])*cos(t4);

    error_code = solveCosineEqn(at,bt,ct,theta2[0],theta2[1]);

    if(error_code != 1)
    {
      continue;
    }

    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
#ifdef DEBUG
      cout << "t2 " << t2 << endl; 
#endif
      sint2 = sin(t2);
      cost2 = cos(t2);

      at = sint1*(ap_[1] - ap_[3])*sint2*sint4;
      bt = (-ap_[1]+ap_[3])*cost1*sint4;
      ct = y - (ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cos(t4)))*sint1;
      error_code = solveCosineEqn(at,bt,ct,theta3[0],theta3[1]);
      if(error_code != 1)
      {
        continue;
      }

      for(int kk =0; kk < 2; kk++)
      {           
        t3 = theta3[kk];
        sint3 = sin(t3);
        cost3 = cos(t3);
#ifdef DEBUG
        cout << "t3 " << t3 << endl; 
#endif
        if(fabs((ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cost4)*sint2+(ap_[1]-ap_[3])*cost2*cost3*sint4-z) > IK_EPS )
          continue;

        if(fabs((ap_[1]-ap_[3])*sint1*sint3*sint4+cost1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - x) > IK_EPS)
          continue;

        grhs_(1,1) = cost4*(gf_(1,1)*cost1*cost2+gf_(2,1)*cost2*sint1-gf_(3,1)*sint2)-(gf_(3,1)*cost2*cost3 + cost3*(gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2 + (-(gf_(2,1)*cost1) + gf_(1,1)*sint1)*sint3)*sint4;

        grhs_(1,2) = cost4*(gf_(1,2)*cost1*cost2 + gf_(2,2)*cost2*sint1 - gf_(3,2)*sint2) - (gf_(3,2)*cost2*cost3 + cost3*(gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2 + (-(gf_(2,2)*cost1) + gf_(1,2)*sint1)*sint3)*sint4;

        grhs_(1,3) = cost4*(gf_(1,3)*cost1*cost2 + gf_(2,3)*cost2*sint1 - gf_(3,3)*sint2) - (gf_(3,3)*cost2*cost3 + cost3*(gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2 + (-(gf_(2,3)*cost1) + gf_(1,3)*sint1)*sint3)*sint4;

        grhs_(2,1) = cost3*(gf_(2,1)*cost1 - gf_(1,1)*sint1) + gf_(3,1)*cost2*sint3 + (gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2*sint3;

        grhs_(2,2) = cost3*(gf_(2,2)*cost1 - gf_(1,2)*sint1) + gf_(3,2)*cost2*sint3 + (gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2*sint3;

        grhs_(2,3) = cost3*(gf_(2,3)*cost1 - gf_(1,3)*sint1) + gf_(3,3)*cost2*sint3 + (gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2*sint3;

        grhs_(3,1) = cost4*(gf_(3,1)*cost2*cost3 + cost3*(gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2 + (-(gf_(2,1)*cost1) + gf_(1,1)*sint1)*sint3) + (gf_(1,1)*cost1*cost2 + gf_(2,1)*cost2*sint1 - gf_(3,1)*sint2)*sint4;

        grhs_(3,2) = cost4*(gf_(3,2)*cost2*cost3 + cost3*(gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2 + (-(gf_(2,2)*cost1) + gf_(1,2)*sint1)*sint3) + (gf_(1,2)*cost1*cost2 + gf_(2,2)*cost2*sint1 - gf_(3,2)*sint2)*sint4;

        grhs_(3,3) = cost4*(gf_(3,3)*cost2*cost3 + cost3*(gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2 + (-(gf_(2,3)*cost1) + gf_(1,3)*sint1)*sint3) + (gf_(1,3)*cost1*cost2 + gf_(2,3)*cost2*sint1 - gf_(3,3)*sint2)*sint4;


        double val1 = sqrt(grhs_(1,2)*grhs_(1,2)+grhs_(1,3)*grhs_(1,3));
        double val2 = grhs_(1,1);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

//            theta6[3] = M_PI + theta6[0];
//            theta6[4] = M_PI + theta6[1];

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
#ifdef DEBUG
          cout << "t6 " << t6 << endl;
#endif
          if(fabs(cos(t6) - grhs_(1,1)) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << endl;
            theta5[0] = acos(grhs_(2,2))/2.0;
            theta7[0] = theta7[0];
            theta7[1] = M_PI+theta7[0];
            theta5[1] = theta7[1];
          }
          else
          {
            theta7[0] = atan2(grhs_(1,2),grhs_(1,3));
            theta5[0] = atan2(grhs_(2,1),-grhs_(3,1));
            theta7[1] = M_PI+theta7[0];
            theta5[1] = M_PI+theta5[0];
          }
#ifdef DEBUG
          std::cout << "theta1: " << t1 << endl;
          std::cout << "theta2: " << t2 << endl;
          std::cout << "theta3: " << t3 << endl;
          std::cout << "theta4: " << t4 << endl;
          std::cout << "theta5: " << t5 << endl;
          std::cout << "theta6: " << t6 << endl;
          std::cout << "theta7: " << t7 << endl << endl << endl;
#endif
          for(int lll =0; lll < 2; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];
#ifdef DEBUG
            cout << "t5" << t5 << endl;
            cout << "t7" << t7 << endl;
#endif      
            if(fabs(sin(t6)*sin(t7)-grhs_(1,2)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(1,3)) > IK_EPS)
              continue;

//                  if(fabs(sin(t5)*sin(t6)-grhs_(2,1)) > IK_EPS || fabs(-cos(t5)*sin(t6)-grhs_(3,1)) > IK_EPS)
//                    continue;


            solution_[0] = normalize_angle(t1)*angle_multipliers_[0];
            solution_[1] = normalize_angle(t2)*angle_multipliers_[1];
            solution_[2] = normalize_angle(t3)*angle_multipliers_[2];
            solution_[3] = normalize_angle(t4)*angle_multipliers_[3];
            solution_[4] = normalize_angle(t5)*angle_multipliers_[4];
            solution_[5] = normalize_angle(t6)*angle_multipliers_[5];
            solution_[6] = normalize_angle(t7)*angle_multipliers_[6];
            solution_ik_.push_back(solution_);

#ifdef DEBUG
            std::cout << t1 << " " << t2 << " " <<  t3 << " " << t4 <<  " " << t5 << " " << t6 <<  " " << t7 << endl << endl;
#endif
          }
        }
      }
    }
  }
}


void arm7DOF::ComputeIKEfficientTheta3(NEWMAT::Matrix g, double t3)
{
  solution_ik_.clear();

  double x = g(1,4);
  double y = g(2,4);
  double z = g(3,4);

/*   for(int i=0; i < 4; i++)
     {
     for(int j=0; j<4; j++)
     {
     wprintf(L" %f ",g(i+1,j+1));
     }
     wprintf(L"\n");
     }
*/
  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost3 = cos(t3);
  sint3 = sin(t3);

  //   t3 = angles::normalize_angle(t3);

  int error_code(0);

  double t1(0), t2(0), t4(0), t5(0), t6(0), t7(0);

  double at(0), bt(0), ct(0);

  double theta1[2],theta2[2],theta4[4],theta5[2],theta6[4],theta7[2];

  double c0 = -sin(-t3)*a4_;
  double c1 = -cos(-t3)*a4_;

  double d0 = 4*a1_*a1_*(a2_*a2_+c1*c1-z*z);
  double d1 = 8*a1_*a1_*a2_*a4_;
  double d2 = 4*a1_*a1_*(a4_*a4_-c1*c1);

  double b0 = x*x+y*y+z*z-a1_*a1_-a2_*a2_-c0*c0-c1*c1;
  double b1 = -2*a2_*a4_;
  /* double denom = b2*b2;

  double dq = (b0*b0-d0)/denom;
  double cq = (2*b1*b0-d1)/denom;
  double bq = (2*b0*b2+b1*b1-d2)/denom;
  double aq = (2*b1*b2)/denom;
  */
  /*  cout << "c0 " << c0 << endl;
      cout << "c1 " << c1 << endl;

      cout << "d0 " << d0 << endl;
      cout << "d1 " << d1 << endl;
      cout << "d2 " << d2 << endl;

      cout << "b0 " << b0 << endl;
      cout << "b1 " << b1 << endl;
      cout << "b2 " << b2 << endl;
  */
  int num_soln = -1;

  int err_code = solve_quadratic(b1*b1-d2,2*b0*b1-d1,b0*b0-d0,&theta4[0],&theta4[1]);
  theta4[0] = acos(theta4[0]);
  theta4[2] = acos(theta4[1]);
  theta4[1] = -theta4[0];
  theta4[3] = -theta4[2];

  if(err_code == 0)
    num_soln = 2;

  if(!num_soln) 
    return;
  for(int jj = 0; jj < 4; jj++)
  {
    t4 = theta4[jj];
    cost4 = cos(t4);
    sint4 = sin(t4);
    //      cout << "t4 " << t4 << endl;
    if(isnan(t4))
      continue;
    at = cos(t3)*sin(t4)*(ap_[1]-ap_[3]);
    bt = (ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cos(t4));
    ct = z;

    error_code = solveCosineEqn(at,bt,ct,theta2[0],theta2[1]);

    if(error_code != 1)
    {
      continue;
    }

    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
      //         cout << "t2 " << t2 << endl; 
#ifdef DEBUG
#endif
      sint2 = sin(t2);
      cost2 = cos(t2);

      at = -y;
      bt = x;
      ct = (ap_[1]-ap_[3])*sin(t3)*sin(t4);
      error_code = solveCosineEqn(at,bt,ct,theta1[0],theta1[1]);
      if(error_code != 1)
      {
        continue;
      }

      for(int kk =0; kk < 2; kk++)
      {           
        t1 = theta1[kk];
        sint1 = sin(t1);
        cost1 = cos(t1);
        //           cout << "t1 " << t1 << endl; 
#ifdef DEBUG
#endif
        if(fabs((ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cost4)*sint2+(ap_[1]-ap_[3])*cost2*cost3*sint4-z) > IK_EPS )
          continue;

        if(fabs((ap_[1]-ap_[3])*sint1*sint3*sint4+cost1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - x) > IK_EPS)
          continue;

        grhs_(1,1) = cost4*(gf_(1,1)*cost1*cost2+gf_(2,1)*cost2*sint1-gf_(3,1)*sint2)-(gf_(3,1)*cost2*cost3 + cost3*(gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2 + (-(gf_(2,1)*cost1) + gf_(1,1)*sint1)*sint3)*sint4;

        grhs_(1,2) = cost4*(gf_(1,2)*cost1*cost2 + gf_(2,2)*cost2*sint1 - gf_(3,2)*sint2) - (gf_(3,2)*cost2*cost3 + cost3*(gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2 + (-(gf_(2,2)*cost1) + gf_(1,2)*sint1)*sint3)*sint4;

        grhs_(1,3) = cost4*(gf_(1,3)*cost1*cost2 + gf_(2,3)*cost2*sint1 - gf_(3,3)*sint2) - (gf_(3,3)*cost2*cost3 + cost3*(gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2 + (-(gf_(2,3)*cost1) + gf_(1,3)*sint1)*sint3)*sint4;

        grhs_(2,1) = cost3*(gf_(2,1)*cost1 - gf_(1,1)*sint1) + gf_(3,1)*cost2*sint3 + (gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2*sint3;

        grhs_(2,2) = cost3*(gf_(2,2)*cost1 - gf_(1,2)*sint1) + gf_(3,2)*cost2*sint3 + (gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2*sint3;

        grhs_(2,3) = cost3*(gf_(2,3)*cost1 - gf_(1,3)*sint1) + gf_(3,3)*cost2*sint3 + (gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2*sint3;

        grhs_(3,1) = cost4*(gf_(3,1)*cost2*cost3 + cost3*(gf_(1,1)*cost1 + gf_(2,1)*sint1)*sint2 + (-(gf_(2,1)*cost1) + gf_(1,1)*sint1)*sint3) + (gf_(1,1)*cost1*cost2 + gf_(2,1)*cost2*sint1 - gf_(3,1)*sint2)*sint4;

        grhs_(3,2) = cost4*(gf_(3,2)*cost2*cost3 + cost3*(gf_(1,2)*cost1 + gf_(2,2)*sint1)*sint2 + (-(gf_(2,2)*cost1) + gf_(1,2)*sint1)*sint3) + (gf_(1,2)*cost1*cost2 + gf_(2,2)*cost2*sint1 - gf_(3,2)*sint2)*sint4;

        grhs_(3,3) = cost4*(gf_(3,3)*cost2*cost3 + cost3*(gf_(1,3)*cost1 + gf_(2,3)*sint1)*sint2 + (-(gf_(2,3)*cost1) + gf_(1,3)*sint1)*sint3) + (gf_(1,3)*cost1*cost2 + gf_(2,3)*cost2*sint1 - gf_(3,3)*sint2)*sint4;


        double val1 = sqrt(grhs_(1,2)*grhs_(1,2)+grhs_(1,3)*grhs_(1,3));
        double val2 = grhs_(1,1);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

//            theta6[3] = M_PI + theta6[0];
//            theta6[4] = M_PI + theta6[1];

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
#ifdef DEBUG
          cout << "t6 " << t6 << endl;
#endif
          if(fabs(cos(t6) - grhs_(1,1)) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << endl;
            theta5[0] = acos(grhs_(2,2))/2.0;
            theta7[0] = theta7[0];
            theta7[1] = M_PI+theta7[0];
            theta5[1] = theta7[1];
          }
          else
          {
            theta7[0] = atan2(grhs_(1,2),grhs_(1,3));
            theta5[0] = atan2(grhs_(2,1),-grhs_(3,1));
            theta7[1] = M_PI+theta7[0];
            theta5[1] = M_PI+theta5[0];
          }
#ifdef DEBUG
          std::cout << "theta1: " << t1 << endl;
          std::cout << "theta2: " << t2 << endl;
          std::cout << "theta3: " << t3 << endl;
          std::cout << "theta4: " << t4 << endl;
          std::cout << "theta5: " << t5 << endl;
          std::cout << "theta6: " << t6 << endl;
          std::cout << "theta7: " << t7 << endl << endl << endl;
#endif
          for(int lll =0; lll < 2; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];
#ifdef DEBUG
            cout << "t5" << t5 << endl;
            cout << "t7" << t7 << endl;
#endif      
            if(fabs(sin(t6)*sin(t7)-grhs_(1,2)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(1,3)) > IK_EPS)
              continue;

//                  if(fabs(sin(t5)*sin(t6)-grhs_(2,1)) > IK_EPS || fabs(-cos(t5)*sin(t6)-grhs_(3,1)) > IK_EPS)
//                    continue;


            solution_[0] = normalize_angle(t1)*angle_multipliers_[0];
            solution_[1] = normalize_angle(t2)*angle_multipliers_[1];
            solution_[2] = normalize_angle(t3)*angle_multipliers_[2];
            solution_[3] = normalize_angle(t4)*angle_multipliers_[3];
            solution_[4] = normalize_angle(t5)*angle_multipliers_[4];
            solution_[5] = normalize_angle(t6)*angle_multipliers_[5];
            solution_[6] = normalize_angle(t7)*angle_multipliers_[6];
            if(CheckJointLimits(solution_))
            {
              solution_ik_.push_back(solution_);
            }
#ifdef DEBUG
            std::cout << t1 << " " << t2 << " " <<  t3 << " " << t4 <<  " " << t5 << " " << t6 <<  " " << t7 << endl << endl;
#endif
          }
        }
      }
    }
  }
}

bool arm7DOF::computeNewGuess(const double &initial_value, double &return_val, int joint_num)
{
  if(negative_increment_valid_)
  {
    if(last_increment_positive_ || !positive_increment_valid_)
    {
      return_val = initial_value - num_negative_increments_ * increment_;
      if(return_val < min_joint_limits_[joint_num])
      {
        negative_increment_valid_ = false;
      }
      else
      {
        last_increment_positive_ = false;
        last_increment_negative_ = true;
        num_negative_increments_++;
        return true;
      }
    }
  }    
  if(positive_increment_valid_)
  {
    if(last_increment_negative_ || !negative_increment_valid_)
    {
      return_val = initial_value + num_positive_increments_ * increment_;
      if(return_val > max_joint_limits_[joint_num])
      {
        positive_increment_valid_ = false;
      }
      else
      {
        last_increment_positive_ = true;
        last_increment_negative_ = false;
        num_positive_increments_++;
        return true;
      }
    }
  }    
  return false;  
}

bool arm7DOF::computeIKFast(NEWMAT::Matrix g, int joint_num, double initial_guess)
{
  if(!(joint_num == 0 || joint_num == 2))
  {
    printf("arm7DOF kinematics: inverse kinematics only supported with joint number 0 or 2 as a free joint");
    return false;
  }

  PrintMatrix(g,"IK::transform");

  double current_guess = initial_guess;

  num_positive_increments_ = 1;
  num_negative_increments_ = 0;

  positive_increment_valid_ = true;
  negative_increment_valid_ = true;
  last_increment_negative_ = false;
  last_increment_positive_ = true;

  if(joint_num == 0)
  {   
    while(1)
    {
      ComputeIKEfficient(g,current_guess);
      if(solution_ik_.size() > 0)
      {
        return true;
      }
      if(!computeNewGuess(initial_guess,current_guess,joint_num))
      {
        return false;
      }
    }
  }


  if(joint_num == 2)
  {   
    while(1)
    {
      ComputeIKEfficientTheta3(g,current_guess);
//      printf("current_guess:: %f\n",current_guess);
      if(solution_ik_.size() > 0)
      {
            return true;
      }
      if(!computeNewGuess(initial_guess,current_guess,joint_num))
      {
        return false;
      }
    }
  }

  return false;

}


bool arm7DOF::computeIKFastWithConstraints(NEWMAT::Matrix g, int joint_num, double initial_guess)
{
  if(!(joint_num == 0 || joint_num == 2))
  {
    printf("arm7DOF kinematics: inverse kinematics only supported with joint number 0 or 2 as a free joint");
    return false;
  }

  PrintMatrix(g,"IK::transform");

  double current_guess = initial_guess;

  num_positive_increments_ = 1;
  num_negative_increments_ = 0;

  positive_increment_valid_ = true;
  negative_increment_valid_ = true;
  last_increment_negative_ = false;
  last_increment_positive_ = true;

  if(joint_num == 0)
  {   
    while(1)
    {
      ComputeIKEfficient(g,current_guess);
      if(solution_ik_.size() > 0)
      {
        return true;
      }
      if(!computeNewGuess(initial_guess,current_guess,joint_num))
      {
        return false;
      }
    }
  }


  if(joint_num == 2)
  {   
    while(1)
    {
      ComputeIKEfficientTheta3(g,current_guess);
//      printf("current_guess:: %f\n",current_guess);
      if(solution_ik_.size() > 0)
      {
        for(int i=0; i<(int)solution_ik_.size(); i++)
        {
          if(solution_ik_[i][1] < 0)
          {
            return true;
          }
        }
      }
      if(!computeNewGuess(initial_guess,current_guess,joint_num))
      {
        return false;
      }
    }
  }

  return false;
}
