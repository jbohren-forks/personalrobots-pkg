#include <libKinematics/kinematics.h>
#include <libKinematics/pr2_ik.h>
#include <sys/time.h>
#include <math.h>
#include <angles/angles.h>

#define EPS_EXACT 0.001

#define FREE_ANGLE_INDEX 2

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

  if(argc < 3)
  {
    cout << "Usage: ./test_ik NUM_TRIALS NUM_DISCRETIZATION" << endl;
    return -1;
  }

  int num_trials = atoi(argv[1]);
  int num_first_angle = atoi(argv[2]);

  srand(time(NULL));

  struct timeval t0,t1;
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
  double angles_check[7] = {0,0,0,0,0,0,0};

  NEWMAT::Matrix g0 = myArm.ComputeFK(angles);
  NEWMAT::Matrix gCheck = myArm.ComputeFK(angles);

  double first_angle(0.0);

  int count_success = 0;
  int count_failure = 0;
  int count_found_solutions = 0;
  int count_found_exact_solutions = 0;
  int count_ik_check = 0;

  bool solved(false);
  double time_taken = 0.0;

  for(int i=0; i < num_trials; i++)
  {
    for(int j=0; j < 7; j++)
    {
      angles[j] = generate_rand_angle();
//       cout << " " << angles[j];
    }
//     cout << endl;

    g0 = myArm.ComputeFK(angles);
    solved = false;
    for(int k=0; k < num_first_angle; k++)
    {
      first_angle = -M_PI + 2*M_PI*k/(double) num_first_angle;
      myArm.ComputeIKEfficientTheta3(g0,first_angle);
      if (myArm.solution_ik_.size() > 0)
      {
        count_found_solutions++;
        for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
        {
          for(int l=0; l < 7; l++)
          {
            angles_check[l] = myArm.solution_ik_[m][l];
          }
          gCheck = myArm.ComputeFK(angles_check);
          if(compareMatrices(gCheck,g0))
          {
            count_success++;
            solved = true;
//             std::cout << "Solved " << k << endl;
          }
          else
          {
            count_ik_check++;
            solved = false;
            cout << "Wrong solution "; 
            for(int l=0; l < 7; l++)
            {
              cout << " " << myArm.solution_ik_[m][l];
            }
            cout << endl;
            cout << gCheck << endl;
            cout << "Input" << endl << g0 << endl;
            double angle_test = myArm.solution_ik_[m][0];
            cout << "Full solution" << endl;
            myArm.ComputeIK(g0,angle_test);
            for(int l=0; l < 7; l++)
            {
              cout << " " << myArm.solution_ik_[0][l];
            }
            cout << endl << endl;
          }
        }
      }
    }
    if(!solved)
    {       
      myArm.ComputeIKEfficientTheta3(g0,angles[FREE_ANGLE_INDEX]);
      if (myArm.solution_ik_.size() > 0)
      {
        for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
        {
          for(int l=0; l < 7; l++)
          {
            angles_check[l] = myArm.solution_ik_[m][l];
          }
          gCheck = myArm.ComputeFK(angles_check);
          if(compareMatrices(gCheck,g0))
          {
            count_success++;
            break;
          }
        }
      }
      else
      {
        count_failure++;
        cout << "Angles " ;
        for(int j=0; j < 7; j++)
        {
          cout << " " << angles[j];
        }
        cout << endl;
        cout << "Parameters " << first_angle << "  " << 2*angles[0]/(double) num_first_angle << endl;

      }
    }

//    std::cout << "Solution exact" << endl << endl << endl;
    bool solution_exact = true;
    gettimeofday(&t0,NULL);
    myArm.ComputeIKEfficientTheta3(g0,angles[FREE_ANGLE_INDEX]);
    gettimeofday(&t1,NULL);
    time_taken += (t1.tv_sec*1000000+t1.tv_usec - (t0.tv_sec*1000000+t0.tv_usec))/1000000.;
//    cout << "Number of solutions: " << myArm.solution_ik_.size() << endl;
    if (myArm.solution_ik_.size() > 0)
    {
      for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
      {
        solution_exact = true;
        for(int l=0; l < 7; l++)
        {
          if(fabs(myArm.solution_ik_[m][l] - angles[l]) > EPS_EXACT)
          {
            solution_exact = false;
            /*            cout << "Solution" << endl;
            for(int jjj = 0; jjj < 7; jjj++)
            {
              cout << " " << myArm.solution_ik_[m][jjj];
            }
            cout << endl << endl;
            */
            break;
          }
        }

        if(solution_exact)
        {
          count_found_exact_solutions++;
          break;
        }
      }
    }

    if(!solution_exact)
      {
        cout << "Solutions for case where exact solution not found" << endl;

        for(int m = 0; m < (int) myArm.solution_ik_.size(); m++)
          {
            for(int jjj = 0; jjj < 7; jjj++)
              {
                cout << " " << myArm.solution_ik_[m][jjj];
              }
            cout << endl << endl;        
          }
      }
         
  }

  cout << "Angles" << endl;
  for(int j=0; j < 7; j++)
  {
    cout << " " << angles[j];
  }
  cout << endl << endl;

  cout << "Input" << endl << g0 << endl;
//  cout << "Success % in IK search: " << (double)count_success/(double)num_trials*100.0 << endl; 
  cout << "Time taken for " << num_trials << " is " << time_taken << " s" << endl;
  cout << "Average time per trial " << time_taken/(double)num_trials << " s" << endl;
  cout << "Number of failures in computing IK: " << count_failure << endl; 
  cout << "Number of failures in matching IK: " << count_ik_check << endl;
  cout << "Number of failures in computing IK exactly: " << num_trials - count_found_exact_solutions << endl; 

}

