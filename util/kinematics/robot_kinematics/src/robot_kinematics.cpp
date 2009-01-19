//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
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

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>
#include <robot_kinematics/robot_kinematics.h>

//#include <mechanism_model/robot.h>

#define eps 0.000001
//#define DEBUG 1

using namespace robot_kinematics;
using namespace KDL;
using namespace std;

RobotKinematics::RobotKinematics():num_chains_(0), chains_(NULL)
{
}

RobotKinematics::~RobotKinematics()
{
  this->serial_chain_map_.clear();
  if (chains_)
    delete[] this->chains_;
}

inline double getMagnitude(double xl[], int num)
{
  int ii;
  double mag = 0;
  for(ii=0; ii < num; ii++)
    mag += (xl[ii]*xl[ii]); 
  return sqrt(mag);
}

void RobotKinematics::loadXMLString(std::string xml_content)
{
  robot_desc::URDF model;
  if(!model.loadString(xml_content.c_str()))
    return;

  loadModel(model);
}

void RobotKinematics::loadXML(std::string filename)
{
  robot_desc::URDF model;
  if(!model.loadFile(filename.c_str()))
    return;

  loadModel(model);
}

bool RobotKinematics::loadString(const char* model_string)
{
  robot_desc::URDF model;
  if(!model.loadString(model_string))
    return false; // urdf fails

  loadModel(model);
  return true;
}

void RobotKinematics::loadModel(const robot_desc::URDF &model)
{
  model.getGroups(groups_);

  for(int i=0; i < (int) groups_.size(); i++)
    if(groups_[i]->hasFlag("kinematic"))
      num_chains_++;

#ifdef DEBUG
  printf("kdl_kinematics.cpp:: num_chains_:: %d\n",num_chains_);
#endif 

  this->chains_ = new SerialChain[num_chains_];  
  this->chain_counter_ = 0;

  for(int i=0; i < (int) groups_.size(); i++)
  {
    if(groups_[i]->hasFlag("kinematic"))
    {
      createChain(groups_[i]);
      printf("Creating kinematic group:: %s \n", groups_[i]->name.c_str());
    }
  }
  printf("Robot kinematics:: done creating all kinematic groups\n");
} 

void RobotKinematics::cross(const double p1[], const double p2[], double p3[])
{
  p3[0] = p1[1]*p2[2]-p1[2]*p2[1];
  p3[1] = p1[2]*p2[0]-p1[0]*p2[2];
  p3[2] = p1[0]*p2[1]-p1[1]*p2[0];
#ifdef DEBUG
  printf("p3: %f %f %f\n",p3[0],p3[1],p3[2]);
#endif
};

double RobotKinematics::getAngleBetweenVectors(double p1[],double p2[])
{
  double dot = p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2];
  double p1_magnitude = getMagnitude(p1,3);
  double p2_magnitude = getMagnitude(p2,3);
  double theta = acos(dot/(p1_magnitude*p2_magnitude)); 

#ifdef DEBUG
  printf("p1: %f %f %f\n",p1[0],p1[1],p1[2]);
  printf("p2: %f %f %f\n",p2[0],p2[1],p2[2]);
  printf("theta: %f\n",theta);
#endif
  return theta;
}

void RobotKinematics::createChain(robot_desc::URDF::Group* group)
{  

  ofstream myfile;
  myfile.open("model.txt");

  KDL::Frame frame1;
  KDL::Frame frame2;

  KDL::Inertia inertia;

  KDL::Frame ident = KDL::Frame::Identity();
  int link_count = 0;

  if((int) group->linkRoots.size() != 1)
  {
    fprintf(stderr,"robot_kinematics.cpp::Too many roots in serial chain!\n");
    return;
  }

  robot_desc::URDF::Link *link_current = group->linkRoots[0];

#ifdef DEBUG
  cout << "root link" << link_current->name << endl;
#endif

  robot_desc::URDF::Link *link_next;

  this->chains_[chain_counter_].link_kdl_frame_ = new KDL::Frame[(int) group->links.size()];
  this->chains_[chain_counter_].name = group->name;
  this->serial_chain_map_[this->chains_[chain_counter_].name] = &(this->chains_[chain_counter_]);

  while(link_count < (int) group->links.size())
  {
    if (link_count < (int) (group->links.size()-1))
    {
      link_next = findNextLinkInGroup(link_current,group);
#ifdef DEBUG
      cout << link_next->name << endl;
#endif
      getKDLJointInXMLFrame(link_current,frame1);
      frame2 = getKDLNextJointFrame(link_current,link_next);
#ifdef DEBUG
      printf("\nComputing and adding frame::%d\n",link_count);
      cout << frame2 << endl << endl << endl;
#endif
    }
    else
    {
      frame1 = ident;
      frame2 = ident;
    }

    KDL::Vector com;
    /* Get inertia matrix and center of mass in KDL frame*/
    inertia = getInertiaInKDLFrame(link_current, com);

    myfile << link_current->name << endl;
    myfile << "KDL Joint in XML Frame" << endl << frame1 << endl << endl;
    myfile << "KDL next joint frame in current KDL joint frame" << endl << frame2 << endl << endl;
    myfile << "Mass " << inertia.m << endl;
    myfile << "Inertia" << endl << inertia.I << endl;
    myfile << "COM " << com << endl << endl;

    this->chains_[chain_counter_].link_kdl_frame_[link_count] = frame1;

    if(link_current->joint->type == robot_desc::URDF::Link::Joint::FIXED)
      this->chains_[chain_counter_].chain.addSegment(Segment(Joint(Joint::None),frame2,inertia,com));
    else        
      this->chains_[chain_counter_].chain.addSegment(Segment(Joint(Joint::RotZ),frame2,inertia,com));

    this->chains_[chain_counter_].joint_id_map_[link_current->name] = link_count + 1;
    link_current = link_next;
    link_count++;
  }
  this->chains_[chain_counter_].finalize();
  chain_counter_++;
  myfile.close();
}

robot_desc::URDF::Link* RobotKinematics::findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group)
{
  std::vector<robot_desc::URDF::Link*>::iterator link_iter;

#ifdef DEBUG
  cout << "Current link:: " << link_current->name << endl; 
#endif

  for(link_iter = link_current->children.begin(); link_iter != link_current->children.end(); link_iter++)
  {
#ifdef DEBUG
    cout << (*link_iter)->name;
#endif
    if((*link_iter)->insideGroup(group))
      return *link_iter;
  }
  return NULL;
}

KDL::Inertia RobotKinematics::getInertiaInKDLFrame(robot_desc::URDF::Link *link_current, KDL::Vector &pos)
{
  KDL::Inertia inertia;
  NEWMAT::Matrix frame(4,4);
  NEWMAT::Matrix I(4,4);
  NEWMAT::Matrix It(4,4);
  NEWMAT::Matrix com_pos(4,1);
  NEWMAT::Matrix com_pos_t(4,1);

  frame = getKDLJointInXMLFrame(link_current);

  com_pos(1,1) = link_current->inertial->com[0];
  com_pos(2,1) = link_current->inertial->com[1];
  com_pos(3,1) = link_current->inertial->com[2];


  cout << "com_pos " << com_pos(1,1) << com_pos(2,1) << com_pos(3,1) << com_pos(4,1) << endl << endl;

  cout << "frame " << endl << frame << endl << endl;
 
  com_pos(4,1) = 1;

  com_pos_t = frame.i() * com_pos;

  frame(1,4) = 0;
  frame(2,4) = 0;
  frame(3,4) = 0;

  I(1,1) = link_current->inertial->inertia[0];
  I(1,2) = link_current->inertial->inertia[1];
  I(1,3) = link_current->inertial->inertia[2];

  I(2,1) = link_current->inertial->inertia[1];
  I(2,2) = link_current->inertial->inertia[3];
  I(2,3) = link_current->inertial->inertia[4];

  I(3,1) = link_current->inertial->inertia[2];
  I(3,2) = link_current->inertial->inertia[4];
  I(3,3) = link_current->inertial->inertia[5];

  It = frame.t() * I * frame; // In general the similarity transform is f I f' but here we are doing f' I f since we are going in the inverse direction.

  inertia.m = link_current->inertial->mass;

  inertia.I.data[0] = It(1,1);
  inertia.I.data[1] = It(1,2);
  inertia.I.data[2] = It(1,3);

  inertia.I.data[3] = It(2,1);
  inertia.I.data[4] = It(2,2);
  inertia.I.data[5] = It(2,3);

  inertia.I.data[6] = It(3,1);
  inertia.I.data[7] = It(3,2);
  inertia.I.data[8] = It(3,3);

  pos[0] = com_pos_t(1,1);
  pos[1] = com_pos_t(2,1);
  pos[2] = com_pos_t(3,1);

  cout << "new " << com_pos_t(1,1) << ", " << com_pos_t(2,1) << ", " << com_pos_t(3,1) << endl;

  return inertia;
/*
  inertia.I.data[0] = link_current->inertial.inertia[0];
  inertia.I.data[1] = link_current->inertial.inertia[1];
  inertia.I.data[2] = link_current->inertial.inertia[2];

  inertia.I.data[3] = link_current->inertial.inertia[1];
  inertia.I.data[4] = link_current->inertial.inertia[3];
  inertia.I.data[5] = link_current->inertial.inertia[4];

  inertia.I.data[6] = link_current->inertial.inertia[2];
  inertia.I.data[7] = link_current->inertial.inertia[4];
  inertia.I.data[8] = link_current->inertial.inertia[5];
*/
}



KDL::Frame RobotKinematics::getKDLNextJointFrame(robot_desc::URDF::Link *link, robot_desc::URDF::Link *link_plus_one)
{
  NEWMAT::Matrix link_link_plus_one(4,4);
  NEWMAT::Matrix link_kdl = getKDLJointInXMLFrame(link);
  NEWMAT::Matrix link_plus_one_kdl_plus_one = getKDLJointInXMLFrame(link_plus_one);

  libTF::Pose3D link_link_plus_one_pose;
  link_link_plus_one_pose.setFromEuler(link_plus_one->xyz[0],link_plus_one->xyz[1],link_plus_one->xyz[2],link_plus_one->rpy[2],link_plus_one->rpy[1],link_plus_one->rpy[0]);
  link_link_plus_one = link_link_plus_one_pose.asMatrix();

  NEWMAT::Matrix result_matrix = link_kdl.i() * link_link_plus_one * link_plus_one_kdl_plus_one;
  KDL::Frame frame = convertNewmatMatrixToKDL(result_matrix);
  return frame;
}

KDL::Frame RobotKinematics::convertNewmatMatrixToKDL(NEWMAT::Matrix m)
{
  KDL::Vector vector(m(1,4),m(2,4),m(3,4));
  KDL::Rotation rotation(m(1,1),m(1,2),m(1,3),m(2,1),m(2,2),m(2,3),m(3,1),m(3,2),m(3,3));
  KDL::Frame frame(rotation,vector);

  return frame;
}

void RobotKinematics::getKDLJointInXMLFrame(robot_desc::URDF::Link *link, KDL::Frame &frame)
{
  NEWMAT::Matrix result_matrix = getKDLJointInXMLFrame(link);
  frame = convertNewmatMatrixToKDL(result_matrix);
//  return convertNewmatMatrixToKDL(result_matrix);
  return;
}

NEWMAT::Matrix RobotKinematics::getKDLJointInXMLFrame(robot_desc::URDF::Link *link)
{
  double p1[3] = {0,0,1};
  double p2[3];
  double p3[3];
  double angle;

  NEWMAT::Matrix lpm(4,4);
  NEWMAT::IdentityMatrix lpi(4);

  libTF::Pose3D local_pose;

  p2[0] = link->joint->axis[0];
  p2[1] = link->joint->axis[1];
  p2[2] = link->joint->axis[2];
      
  if(fabs(p2[2] > (1-eps)))
    return lpi;

  cross(p1,p2,p3);//get axis
  angle = getAngleBetweenVectors(p1,p2);

  local_pose.setAxisAngle(p3,angle);

  lpm = local_pose.asMatrix();
  lpm(1,4) = link->joint->anchor[0];
  lpm(2,4) = link->joint->anchor[1];
  lpm(3,4) = link->joint->anchor[2];

  return lpm;
}

SerialChain* RobotKinematics::getSerialChain(std::string name) const
{
  std::map<std::string, SerialChain*>::const_iterator it = serial_chain_map_.find(name);
  return (it == serial_chain_map_.end()) ? NULL : it->second;
}

/*
  void RobotKinematics::mapRobotModelJoints(Robot *r)
  {
  for(int i=0; i < this->num_chains_; i++)
  {
  for(int j=0; j < this->chains_[i]->num_joints_; j++)
  {
  this->chains_[i]->joints_[j] = r->getJoint(this->chains_[i]->joint_names_[j]);      
  }
  }
  }

  double RobotKinematics::update(Robot *r)
  {
  
  }
*/

double RobotKinematics::subProblem1(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix w)
{
  NEWMAT::Matrix u(3,1),v(3,1),up(3,1),vp(3,1);
  NEWMAT::Matrix p(3,1),q(3,1),r(3,1);

  NEWMAT::Matrix arg1, arg2;

  p = pin.SubMatrix(1,3,1,1);
  q = qin.SubMatrix(1,3,1,1);
  r = rin.SubMatrix(1,3,1,1);

#ifdef DEBUG
  PrintMatrix(p,"subProblem1::p");
  PrintMatrix(q,"subProblem1::q");
  PrintMatrix(r,"subProblem1::r");
  PrintMatrix(w,"subProblem1::w");
#endif

  u = p-r;
  v = q-r;

  up = u - w*(w.t()*u);
  vp = v - w*(w.t()*v);

#ifdef DEBUG
  PrintMatrix(up,"SP1::up::");
  PrintMatrix(vp,"SP1::vp::");
  PrintMatrix(w,"SP1::w");
#endif

  arg1 = w.t()*cross(up,vp);
  arg2 = up.t()*vp;

#ifdef DEBUG
  cout << "subProblem1::" << arg1(1,1) << "," << arg2(1,1) << endl;
  cout << endl << endl << endl;
#endif
  return atan2(arg1(1,1),arg2(1,1));
}

NEWMAT::Matrix RobotKinematics::cross(NEWMAT::Matrix p1, NEWMAT::Matrix p2)
{
  NEWMAT::Matrix p3(3,1);
  p3(1,1) = p1(2,1)*p2(3,1)-p1(3,1)*p2(2,1);
  p3(2,1) = p1(3,1)*p2(1,1)-p1(1,1)*p2(3,1);
  p3(3,1) = p1(1,1)*p2(2,1)-p1(2,1)*p2(1,1);

  return p3;
};

void RobotKinematics::subProblem2(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix omega1, NEWMAT::Matrix omega2, double theta[])
{
  NEWMAT::Matrix u,v,z,c;
  NEWMAT::Matrix p,q,r;
  NEWMAT::Matrix denom;

  NEWMAT::Matrix alpha,beta,gamma_numerator,gamma_denominator;

  double gamma;

  p = pin.SubMatrix(1,3,1,1);
  q = qin.SubMatrix(1,3,1,1);
  r = rin.SubMatrix(1,3,1,1);

  u = p-r;
  v = q-r;

  denom = omega1.t()*omega2;

#ifdef DEBUG
  PrintMatrix(p,"SubProblem2::p");
  PrintMatrix(q,"SubProblem2::q");
  PrintMatrix(u,"SubProblem2::u");
  PrintMatrix(v,"SubProblem2::v");
  PrintMatrix(denom,"SubProblem2::denom");
#endif

  alpha = ((omega1.t()*omega2)*(omega2.t()*u)-(omega1.t()*v))/(pow(denom(1,1),2)-1);
  beta = ((omega1.t()*omega2)*(omega1.t()*v)-(omega2.t()*u))/(pow(denom(1,1),2)-1);

#ifdef DEBUG
  PrintMatrix(alpha,"SubProblem2::alpha");
  PrintMatrix(beta,"SubProblem2::beta");
#endif

  gamma_numerator = ((u.t()*u)-pow(alpha(1,1),2)-pow(beta(1,1),2)-2*alpha(1,1)*beta(1,1)*(omega1.t()*omega2));
  gamma_denominator = (cross(omega1,omega2).t()*cross(omega1,omega2));
  gamma = sqrt(gamma_numerator(1,1)/gamma_denominator(1,1));


#ifdef DEBUG
  cout << "SubProblem2::gamma" << endl << gamma << endl << endl;
#endif

  z = alpha(1,1)*omega1 + beta(1,1)*omega2 + gamma*cross(omega1,omega2);
  c = z+r;

#ifdef DEBUG
  PrintMatrix(c,"SubProblem2::c");
  PrintMatrix(z,"SubProblem2::z");
#endif

  theta[0] = subProblem1(c,q,r,omega1);
  theta[1] = subProblem1(p,c,r,omega2);

  z = alpha(1,1)*omega1 + beta(1,1)*omega2 - gamma*cross(omega1,omega2);
  c = z+r;

#ifdef DEBUG
  PrintMatrix(c,"SubProblem2::c");
  PrintMatrix(z,"SubProblem2::z");
#endif

  theta[2] = subProblem1(c,q,r,omega1);
  theta[3] = subProblem1(p,c,r,omega2);
}


