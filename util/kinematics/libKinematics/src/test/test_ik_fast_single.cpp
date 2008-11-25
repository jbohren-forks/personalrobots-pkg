#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>
#include <sys/time.h>
#include <math.h>
#include <angles/angles.h>

#define EPS_EXACT 0.001

using namespace kinematics;
using namespace std;

double generate_rand_angle()
{
  double result = ((double)(rand()-RAND_MAX/2))/(double)(RAND_MAX/2)*M_PI;
  result = angles::normalize_angle(result);
  return result;
}

bool compareMatrices(NEWMAT::Matrix g0, NEWMAT::Matrix g1)
{
  NEWMAT::Matrix gd = g1 - g0;
  if(gd.MaximumAbsoluteValue() < 0.001)
  {
    return true;
  }
  else
    return false;
}

int main(int argc, char** argv)
{

  srand(time(NULL));

  NEWMAT::Matrix g(4,4);
  g = 0;

  std::vector<NEWMAT::Matrix> axis;
  std::vector<NEWMAT::Matrix> anchor;
  std::vector<std::string> joint_type;

  NEWMAT::Matrix aj(3,1);
  NEWMAT::Matrix an(3,1);

  joint_type.resize(7);

  // Shoulder pan
  aj << 0 << 0 << 1.0;
  axis.push_back(aj);
  an << 0 << 0 << 0;
  anchor.push_back(an);

  // Shoulder pitch
  aj << 0 << 1.0 << 0;
  axis.push_back(aj);
  an << 0.1 << 0 << 0;
  anchor.push_back(an);

  // Shoulder roll
  aj << 1.0 << 0 << 0;
  axis.push_back(aj);
  an << 0.1 << 0 << 0;
  anchor.push_back(an);

  // Elbow flex
  aj << 0 << 1.0 << 0;
  axis.push_back(aj);
  an << 0.5 << 0 << 0;
  anchor.push_back(an);

  // Forearm roll
  aj << 1.0 << 0 << 0;
  axis.push_back(aj);
  an << 0.5 << 0 << 0;
  anchor.push_back(an);

  // Wrist flex
  aj << 0 << 1.0 << 0;
  axis.push_back(aj);
  an << 0.82025 << 0 << 0;
  anchor.push_back(an);

  // Gripper roll
  aj << 1.0 << 0 << 0;
  axis.push_back(aj);
  an << 0.82025 << 0 << 0;
  anchor.push_back(an);

  for(int i=0; i < 7; i++)
    joint_type[i] = std::string("ROTARY");

  arm7DOF myArm(anchor,axis,joint_type);

/* Joint angles and speeds for testing */
  double angles[7] = {0,0,0,0,0,0,0};
  double angles_check[7] = { -1.42361,1.359057, -1.5, 1.2, -2.4035 ,1.86428 ,0.118674};

  NEWMAT::Matrix g0 = myArm.ComputeFK(angles);
  NEWMAT::Matrix gCheck = myArm.ComputeFK(angles);

//  NEWMAT::Matrix g0(4,4);
//  g0 << -0.297262 << -0.835609 << 0.461945 << 0.441604 << -0.440776 << 0.549275 << 0.709939 << 0.621504 << -0.846966 << 0.007424 << -0.531595 <<  0.101515 << 0.000000 << 0.000000 << 0.000000 << 1.000000; 


  int count_success = 0;
  int count_found_exact_solutions = 0;
  bool solution_exact = true;

  for(int j=0; j < 7; j++)
  {
    angles[j] = angles_check[j];
  }

  g0 = myArm.ComputeFK(angles);

  cout << "Angles: " ;
  for(int l=0; l < 7; l++)
  {
    cout << " " << angles[l];
  }
  cout << endl;


  myArm.ComputeIKEfficientTheta3(g0,-1.5);
  cout << "Input" << endl << g0 << endl << endl;
  cout << "Solution" << endl << endl;
  for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
  {
    for(int l=0; l < 7; l++)
    {
      cout << " " << myArm.solution_ik_[m][l];
    }
    cout << endl;
  }
  if (myArm.solution_ik_.size() > 0)
  {
    for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
    {
      solution_exact = true;
      for(int l=0; l < 7; l++)
      {
        angles_check[l] = myArm.solution_ik_[m][l];
      }
      gCheck = myArm.ComputeFK(angles_check);
      if(compareMatrices(gCheck,g0))
      {
        count_success++;
      }
    for(int l=0; l < 7; l++)
      {
        if(fabs(myArm.solution_ik_[m][l] - angles[l]) > EPS_EXACT)
        {
          solution_exact = false;
          break;
        }
      }
      if(solution_exact)
      {
        count_found_exact_solutions++;
      }

    }
  }
  cout << "Success for " << count_success << " of " << myArm.solution_ik_.size() << " solutions" << endl;
  cout << "Found " << count_found_exact_solutions <<  " exact solution" << endl;
}

