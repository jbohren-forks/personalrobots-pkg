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

#include <angles/angles.h>
#include <pr2_ik/pr2_ik.h>

/**** List of angles (for reference) *******
      th1 = shoulder/turret pan
      th2 = shoulder/turret lift/pitch
      th3 = shoulder/turret roll
      th4 = elbow pitch
      th5 = elbow roll 
      th6 = wrist pitch
      th7 = wrist roll 
*****/

//#define DEBUG 1
using namespace pr2_ik;
using namespace angles;

PR2IK::PR2IK(const double &shoulder_offset_distance, 
             const double &shoulder_elbow_distance, 
             const double &elbow_wrist_distance, 
             const double &torso_shoulder_x, 
             const double &torso_shoulder_y, 
             const double &torso_shoulder_z, 
             const int &free_angle)
{
  angle_multipliers_.resize(NUM_JOINTS_ARM7DOF);
  solution_.resize(NUM_JOINTS_ARM7DOF);
  min_angles_.resize(NUM_JOINTS_ARM7DOF);
  max_angles_.resize(NUM_JOINTS_ARM7DOF);

  /* Internal variables */
  if(!setFreeAngle(free_angle))
    free_angle_ = 2;

  for(int i=0; i< NUM_JOINTS_ARM7DOF; i++)
  {
    angle_multipliers_[i] = 1.0;
    min_angles_[i] = -M_PI;
    max_angles_[i] = M_PI;
  }
  Eigen::Matrix4f home = Eigen::Matrix4f::Identity();
  home(0,3) = shoulder_offset_distance +  shoulder_elbow_distance +  elbow_wrist_distance;
  home_inv_ = matrixInverse(home);

  std::cout << "Home inverse " << home_inv_ << std::endl;
  grhs_ = home;
  gf_ = home_inv_;
  a1_ = shoulder_offset_distance;
  a2_ = shoulder_elbow_distance;
  a4_ = elbow_wrist_distance;

  ap_[0] = a1_;
  ap_[1] = a1_+a2_;
  ap_[2] = a1_+a2_+a4_;
  ap_[3] = a1_+a2_+a4_;
  ap_[4] = a1_+a2_+a4_;
  torso_shoulder_offset_ = Eigen::Vector3f::Zero();
  torso_shoulder_offset_(0) = torso_shoulder_x;
  torso_shoulder_offset_(1) = torso_shoulder_y;
  torso_shoulder_offset_(2) = torso_shoulder_z;
}

PR2IK::~PR2IK()
{
}

bool PR2IK::setJointLimits(std::vector<double> min_angles, std::vector<double> max_angles)
{
  if(min_angles.size() != min_angles_.size() || max_angles.size() != max_angles_.size())
  {
    printf("Size of input angles %d does not match number of joints %d",(int)min_angles.size(),(int) min_angles_.size());
    return false;
  }
  for(int i=0; i < NUM_JOINTS_ARM7DOF; i++)
  {
    min_angles_[i] = min_angles[i];
    max_angles_[i] = max_angles[i];
  }
  return true;
}

bool PR2IK::setFreeAngle(int angle_index)
{
  if(angle_index != 0 && angle_index != 2) 
    return false;

  free_angle_ = angle_index;
  return true;
}

bool PR2IK::setAngleMultipliers(std::vector<double> angle_mult)
{
  if(angle_mult.size() != angle_multipliers_.size())
  {
    printf("Size of input vector %d does not match number of joints %d",(int) angle_mult.size(),(int) angle_multipliers_.size());
    return false;
  }
  printf("Angle multipliers ");
  for(int i=0; i< (int) angle_multipliers_.size(); i++)
  {
    angle_multipliers_[i] = angle_mult[i];
    printf("%f ",angle_multipliers_[i]);
  }
  printf("\n");
//  angle_multipliers_[5] = 1.0;
  return true;
}

bool PR2IK::solveQuadratic(const double &a, const double &b, const double &c, double *x1, double *x2)
{
  double discriminant = b*b-4*a*c;
  if(fabs(a) < IK_EPS)
  {
    *x1 = -c/b;
    *x2 = *x1;
    return true;
  }
#ifdef DEBUG
  printf("Discriminant: %f\n",discriminant);
#endif
  if (discriminant >= 0)
  {      
    *x1 = (-b + sqrt(discriminant))/(2*a); 
    *x2 = (-b - sqrt(discriminant))/(2*a);
    return true;
  } 
  else if(fabs(discriminant) < IK_EPS)
  {
    *x1 = -b/(2*a);
    *x2 = -b/(2*a);
    return true;
  }
  else
  {
    *x1 = -b/(2*a);
    *x2 = -b/(2*a);
    return false;
  }
}

Eigen::Matrix4f PR2IK::matrixInverse(const Eigen::Matrix4f &g)
{
  Eigen::Matrix4f result = g;
  Eigen::Matrix3f Rt = Eigen::Matrix3f::Identity();

  Eigen::Vector3f p = Eigen::Vector3f::Zero(3);
  Eigen::Vector3f pinv = Eigen::Vector3f::Zero(3);

  Rt(0,0) = g(0,0);
  Rt(1,1) = g(1,1);
  Rt(2,2) = g(2,2);

  Rt(0,1) = g(1,0);
  Rt(1,0) = g(0,1);

  Rt(0,2) = g(2,0);
  Rt(2,0) = g(0,2);

  Rt(1,2) = g(2,1);
  Rt(2,1) = g(1,2);

  p(0) = g(0,3);
  p(1) = g(1,3);
  p(2) = g(2,3);

  pinv = -Rt*p;

  result(0,0) = g(0,0);
  result(1,1) = g(1,1);
  result(2,2) = g(2,2);

  result(0,1) = g(1,0);
  result(1,0) = g(0,1);

  result(0,2) = g(2,0);
  result(2,0) = g(0,2);

  result(1,2) = g(2,1);
  result(2,1) = g(1,2);

  result(0,3) = pinv(0);
  result(1,3) = pinv(1);
  result(2,3) = pinv(2);
  
  return result;
}


bool PR2IK::solveCosineEqn(const double &a, const double &b, const double &c, double &soln1, double &soln2)
{
  double theta1 = atan2(b,a);
  double denom  = sqrt(a*a+b*b);

  if(fabs(denom) < IK_EPS) // should never happen, wouldn't make sense but make sure it is checked nonetheless
  {
#ifdef DEBUG
    std::cout << "denom: " << denom << std::endl;
#endif
    return false;
  }
  double rhs_ratio = c/denom;

//  if(fabs(rhs_ratio)-1 < 0.01)
//  { 
//    rhs_ratio = std::min<double>(std::max<double>(rhs_ratio,-1.0),1.0);
//  }
//  else 
  if(rhs_ratio < -1 || rhs_ratio > 1)
  {
#ifdef DEBUG
    std::cout << "rhs_ratio: " << rhs_ratio << std::endl;
#endif
    return false;
  }
  double acos_term = acos(rhs_ratio);
  soln1 = theta1 + acos_term;
  soln2 = theta1 - acos_term;

  return true;
}



bool PR2IK::checkJointLimits(const std::vector<double> &joint_values)
{
  for(int i=0; i<NUM_JOINTS_ARM7DOF; i++)
  {
     if(!checkJointLimits(angles::normalize_angle(joint_values[i]*angle_multipliers_[i]),i))
    {
      return false;
    }
    }
  return true;
}

bool PR2IK::checkJointLimits(const double &joint_value, const int &joint_num)
{

   double jv = angles::normalize_angle(joint_value*angle_multipliers_[joint_num]);
  if(jv < min_angles_[joint_num] || jv > max_angles_[joint_num])
  {
#ifdef DEBUG
   printf("Angle %d = %f out of range: (%f,%f)\n",joint_num,joint_value,min_angles_[joint_num],max_angles_[joint_num]);
#endif
    return false;
  }
  return true;
}


void PR2IK::computeIKEfficient(const Eigen::Matrix4f &g_in, const double &t1_in)
{
//t1 = shoulder/turret pan is specified
  solution_ik_.clear();

  Eigen::Matrix4f g = g_in;
//First bring everything into the arm frame
  g(0,3) = g_in(0,3) - torso_shoulder_offset_(0);
  g(1,3) = g_in(1,3) - torso_shoulder_offset_(1);
  g(2,3) = g_in(2,3) - torso_shoulder_offset_(2);

  double t1 = angles::normalize_angle(t1);
  if(!checkJointLimits(t1,0))
    return;


  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost1 = cos(t1);
  sint1 = sin(t1);

  double t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);

  double at(0), bt(0), ct(0);

  double theta2[2],theta3[2],theta4[2],theta5[2],theta6[4],theta7[2]; 

  double sopx = ap_[0]*cost1;
  double sopy = ap_[0]*sint1;
  double sopz = 0;

  double x = g(0,3);
  double y = g(1,3);
  double z = g(2,3);

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
  std::cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << std::endl << theta4[0] << std::endl;
#endif

  for(int jj =0; jj < 2; jj++)
  {
    t4 = theta4[jj];
    cost4 = cos(t4);
    sint4 = sin(t4);

#ifdef DEBUG
    std::cout << "t4 " << t4 << std::endl;
#endif
    if(isnan(t4))
      continue;

    if(!checkJointLimits(t4,3))
      continue;

    at = x*cost1+y*sint1-ap_[0];
    bt = -z;
    ct = -ap_[0] + ap_[1] + (ap_[3]-ap_[1])*cos(t4);

    if(!solveCosineEqn(at,bt,ct,theta2[0],theta2[1]))
      continue;

    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
      if(!checkJointLimits(t2,1))
        continue;


#ifdef DEBUG
      std::cout << "t2 " << t2 << std::endl; 
#endif
      sint2 = sin(t2);
      cost2 = cos(t2);

      at = sint1*(ap_[1] - ap_[3])*sint2*sint4;
      bt = (-ap_[1]+ap_[3])*cost1*sint4;
      ct = y - (ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cos(t4)))*sint1;
      if(!solveCosineEqn(at,bt,ct,theta3[0],theta3[1]))
        continue;

      for(int kk =0; kk < 2; kk++)
      {           
        t3 = theta3[kk];

        if(!checkJointLimits(angles::normalize_angle(t3),2))
          continue;

        sint3 = sin(t3);
        cost3 = cos(t3);
#ifdef DEBUG
        std::cout << "t3 " << t3 << std::endl; 
#endif
        if(fabs((ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cost4)*sint2+(ap_[1]-ap_[3])*cost2*cost3*sint4-z) > IK_EPS )
          continue;

        if(fabs((ap_[1]-ap_[3])*sint1*sint3*sint4+cost1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - x) > IK_EPS)
          continue;

        grhs_(0,0) = cost4*(gf_(0,0)*cost1*cost2+gf_(1,0)*cost2*sint1-gf_(2,0)*sint2)-(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3)*sint4;

        grhs_(0,1) = cost4*(gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2) - (gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3)*sint4;

        grhs_(0,2) = cost4*(gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2) - (gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3)*sint4;

        grhs_(1,0) = cost3*(gf_(1,0)*cost1 - gf_(0,0)*sint1) + gf_(2,0)*cost2*sint3 + (gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2*sint3;

        grhs_(1,1) = cost3*(gf_(1,1)*cost1 - gf_(0,1)*sint1) + gf_(2,1)*cost2*sint3 + (gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2*sint3;

        grhs_(1,2) = cost3*(gf_(1,2)*cost1 - gf_(0,2)*sint1) + gf_(2,2)*cost2*sint3 + (gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2*sint3;

        grhs_(2,0) = cost4*(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3) + (gf_(0,0)*cost1*cost2 + gf_(1,0)*cost2*sint1 - gf_(2,0)*sint2)*sint4;

        grhs_(2,1) = cost4*(gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3) + (gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2)*sint4;

        grhs_(2,2) = cost4*(gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3) + (gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2)*sint4;


        double val1 = sqrt(grhs_(0,1)*grhs_(0,1)+grhs_(0,2)*grhs_(0,2));
        double val2 = grhs_(0,0);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

//            theta6[3] = M_PI + theta6[0];
//            theta6[4] = M_PI + theta6[1];

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
          if(!checkJointLimits(angles::normalize_angle(t6),5))
            continue;

#ifdef DEBUG
          std::cout << "t6 " << t6 << std::endl;
#endif
          if(fabs(cos(t6) - grhs_(0,0)) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << std::endl;
            theta5[0] = acos(grhs_(1,1))/2.0;
            theta7[0] = theta7[0];
            theta7[1] = M_PI+theta7[0];
            theta5[1] = theta7[1];
          }
          else
          {
            theta7[0] = atan2(grhs_(0,1),grhs_(0,2));
            theta5[0] = atan2(grhs_(1,0),-grhs_(2,0));
            theta7[1] = M_PI+theta7[0];
            theta5[1] = M_PI+theta5[0];
          }
#ifdef DEBUG
          std::cout << "theta1: " << t1 << std::endl;
          std::cout << "theta2: " << t2 << std::endl;
          std::cout << "theta3: " << t3 << std::endl;
          std::cout << "theta4: " << t4 << std::endl;
          std::cout << "theta5: " << t5 << std::endl;
          std::cout << "theta6: " << t6 << std::endl;
          std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
#endif
          for(int lll =0; lll < 2; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];
            if(!checkJointLimits(t5,4))
              continue;
            if(!checkJointLimits(t7,6))
              continue;

#ifdef DEBUG
            std::cout << "t5" << t5 << std::endl;
            std::cout << "t7" << t7 << std::endl;
#endif      
            if(fabs(sin(t6)*sin(t7)-grhs_(0,1)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(0,2)) > IK_EPS)
              continue;

            solution_[0] = normalize_angle(t1)*angle_multipliers_[0];
            solution_[1] = normalize_angle(t2)*angle_multipliers_[1];
            solution_[2] = normalize_angle(t3)*angle_multipliers_[2];
            solution_[3] = normalize_angle(t4)*angle_multipliers_[3];
            solution_[4] = normalize_angle(t5)*angle_multipliers_[4];
            solution_[5] = normalize_angle(t6)*angle_multipliers_[5];
            solution_[6] = normalize_angle(t7)*angle_multipliers_[6];
            solution_ik_.push_back(solution_);

#ifdef DEBUG
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#endif
          }
        }
      }
    }
  }
}


void PR2IK::computeIKEfficientTheta3(const Eigen::Matrix4f &g_in, const double &t3)
{
  solution_ik_.clear();
//t3 = shoulder/turret roll is specified
  Eigen::Matrix4f g = g_in;
//First bring everything into the arm frame
  g(0,3) = g_in(0,3) - torso_shoulder_offset_(0);
  g(1,3) = g_in(1,3) - torso_shoulder_offset_(1);
  g(2,3) = g_in(2,3) - torso_shoulder_offset_(2);

  if(!checkJointLimits(t3,2))
  {
    return;
  }
  double x = g(0,3);
  double y = g(1,3);
  double z = g(2,3);
  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  gf_ = g*home_inv_;

  cost3 = cos(t3);
  sint3 = sin(t3);

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

  if(!solveQuadratic(b1*b1-d2,2*b0*b1-d1,b0*b0-d0,&theta4[0],&theta4[1]))
  {
#ifdef DEBUG
     printf("No solution to quadratic eqn\n");
#endif
    return;
  }
  theta4[0] = acos(theta4[0]);
  theta4[2] = acos(theta4[1]);
  theta4[1] = -theta4[0];
  theta4[3] = -theta4[2];

  for(int jj = 0; jj < 4; jj++)
  {
    t4 = theta4[jj];

    if(!checkJointLimits(t4,3))
    {
      continue;
    }
    cost4 = cos(t4);
    sint4 = sin(t4);
#ifdef DEBUG
     std::cout << "t4 " << t4 << std::endl;
#endif
    if(isnan(t4))
      continue;
    at = cos(t3)*sin(t4)*(ap_[1]-ap_[3]);
    bt = (ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cos(t4));
    ct = z;

    if(!solveCosineEqn(at,bt,ct,theta2[0],theta2[1]))
      continue;

    for(int ii=0; ii < 2; ii++)
    {
      t2 = theta2[ii];
#ifdef DEBUG
     std::cout << "t2 " << t2 << std::endl;
#endif
      if(!checkJointLimits(t2,1))
      {
        continue;
      }


      sint2 = sin(t2);
      cost2 = cos(t2);

      at = -y;
      bt = x;
      ct = (ap_[1]-ap_[3])*sin(t3)*sin(t4);
      if(!solveCosineEqn(at,bt,ct,theta1[0],theta1[1]))
      {
#ifdef DEBUG
        std::cout << "could not solve cosine equation for t1" << std::endl;
#endif
        continue;
      }

      for(int kk =0; kk < 2; kk++)
      {           
        t1 = theta1[kk];
#ifdef DEBUG
        std::cout << "t1 " << t1 << std::endl;
#endif
        if(!checkJointLimits(t1,0))
        {
          continue;
        }
        sint1 = sin(t1);
        cost1 = cos(t1);
        if(fabs((ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cost4)*sint2+(ap_[1]-ap_[3])*cost2*cost3*sint4-z) > IK_EPS )
        {
#ifdef DEBUG
           printf("z value not matched %f\n",fabs((ap_[0]-ap_[1]+(ap_[1]-ap_[3])*cost4)*sint2+(ap_[1]-ap_[3])*cost2*cost3*sint4-z));
#endif
          continue;
        }
        if(fabs((ap_[1]-ap_[3])*sint1*sint3*sint4+cost1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - x) > IK_EPS)
        {
#ifdef DEBUG
           printf("x value not matched by %f\n",fabs((ap_[1]-ap_[3])*sint1*sint3*sint4+cost1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - x));
#endif
          continue;
        }
        if(fabs(-(ap_[1]-ap_[3])*cost1*sint3*sint4+sint1*(ap_[0]+cost2*(-ap_[0]+ap_[1]+(-ap_[1]+ap_[3])*cost4)+(ap_[1]-ap_[3])*cost3*sint2*sint4) - y) > IK_EPS)
        {
#ifdef DEBUG
           printf("y value not matched\n");
#endif
           continue;
        }
        grhs_(0,0) = cost4*(gf_(0,0)*cost1*cost2+gf_(1,0)*cost2*sint1-gf_(2,0)*sint2)-(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3)*sint4;

        grhs_(0,1) = cost4*(gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2) - (gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3)*sint4;

        grhs_(0,2) = cost4*(gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2) - (gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3)*sint4;

        grhs_(1,0) = cost3*(gf_(1,0)*cost1 - gf_(0,0)*sint1) + gf_(2,0)*cost2*sint3 + (gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2*sint3;

        grhs_(1,1) = cost3*(gf_(1,1)*cost1 - gf_(0,1)*sint1) + gf_(2,1)*cost2*sint3 + (gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2*sint3;

        grhs_(1,2) = cost3*(gf_(1,2)*cost1 - gf_(0,2)*sint1) + gf_(2,2)*cost2*sint3 + (gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2*sint3;

        grhs_(2,0) = cost4*(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3) + (gf_(0,0)*cost1*cost2 + gf_(1,0)*cost2*sint1 - gf_(2,0)*sint2)*sint4;

        grhs_(2,1) = cost4*(gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3) + (gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2)*sint4;

        grhs_(2,2) = cost4*(gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3) + (gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2)*sint4;









        double val1 = sqrt(grhs_(0,1)*grhs_(0,1)+grhs_(0,2)*grhs_(0,2));
        double val2 = grhs_(0,0);

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);

        for(int mm = 0; mm < 2; mm++)
        {
          t6 = theta6[mm];
#ifdef DEBUG
          std::cout << "t6 " << t6 << std::endl;
#endif
          if(!checkJointLimits(t6,5))
          {
            continue;
          }


          if(fabs(cos(t6) - grhs_(0,0)) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS)
          {
            //                std::cout << "Singularity" << std::endl;
            theta5[0] = acos(grhs_(1,1))/2.0;
            theta7[0] = theta5[0];
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = theta7[1];
          }
          else
          {
             theta7[0] = atan2(grhs_(0,1)/sin(t6),grhs_(0,2)/sin(t6));
             theta5[0] = atan2(grhs_(1,0)/sin(t6),-grhs_(2,0)/sin(t6));
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = M_PI+theta5[0];
          }
          for(int lll =0; lll < 1; lll++)
          {
            t5 = theta5[lll];
            t7 = theta7[lll];

            if(!checkJointLimits(t5,4))
            {
              continue;
            }
            if(!checkJointLimits(t7,6))
            {
              continue;
            }


#ifdef DEBUG
            std::cout << "t5 " << t5 << std::endl;
            std::cout << "t7 " << t7 << std::endl;
#endif      
            //           if(fabs(sin(t6)*sin(t7)-grhs_(0,1)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(0,2)) > IK_EPS)
            //  continue;

#ifdef DEBUG
          std::cout << "theta1: " << t1 << std::endl;
          std::cout << "theta2: " << t2 << std::endl;
          std::cout << "theta3: " << t3 << std::endl;
          std::cout << "theta4: " << t4 << std::endl;
          std::cout << "theta5: " << t5 << std::endl;
          std::cout << "theta6: " << t6 << std::endl;
          std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
#endif


          solution_[0] = normalize_angle(t1*angle_multipliers_[0]);
          solution_[1] = normalize_angle(t2*angle_multipliers_[1]);
          solution_[2] = normalize_angle(t3*angle_multipliers_[2]);
          solution_[3] = normalize_angle(t4*angle_multipliers_[3]);
        solution_[4] = normalize_angle(t5*angle_multipliers_[4]);
      solution_[5] = normalize_angle(t6*angle_multipliers_[5]);
    solution_[6] = normalize_angle(t7*angle_multipliers_[6]);
            solution_ik_.push_back(solution_);
#ifdef DEBUG
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#endif
          }
        }
      }
    }
  }
}
