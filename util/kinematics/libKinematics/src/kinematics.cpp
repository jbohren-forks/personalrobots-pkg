#include <math.h>
#include <libKinematics/kinematics.h>

#define EPS 0.000001
using namespace kinematics;

void kinematics::PrintMatrix(NEWMAT::Matrix m, std::string c)
{
   cout << c << endl << m << endl << endl;
}

NEWMAT::Matrix kinematics::GetHatMatrix(NEWMAT::Matrix omega)
{
   NEWMAT::Matrix result(3,3);

   result = 0;

   result(1,2) = -omega(3,1);
   result(1,3) = omega(2,1);
   result(2,1) = omega(3,1);
   result(2,3) = -omega(1,1);
   result(3,1) = -omega(2,1);
   result(3,2) = omega(1,1);

   return result;
};

NEWMAT::Matrix kinematics::ExpRot(NEWMAT::Matrix omega, double theta)
{
   NEWMAT::Matrix Rot(3,3);
   NEWMAT::IdentityMatrix I(3);
   NEWMAT::Matrix omegaHat(3,3);
   double omegaMag;

   omegaHat = GetHatMatrix(omega);
   omegaMag = GetNorm(omega);

   if (omegaMag < EPS)
      Rot = I;
   else
      Rot = I + omegaHat*(sin(omegaMag*theta)/omegaMag) + (omegaHat*omegaHat)*(1-cos(omegaMag*theta))/(omegaMag*omegaMag);
#ifdef DEBUG  
   cout << omega;
   cout << "ExpRot" << endl << Rot << endl;
#endif
   return Rot;
};

double kinematics::GetNorm(NEWMAT::Matrix omega)
{
   double norm(0);
   norm = sqrt(omega(1,1)*omega(1,1)+omega(2,1)*omega(2,1)+omega(3,1)*omega(3,1));
   return norm;
};

NEWMAT::Matrix kinematics::ExpTwist(NEWMAT::Matrix xi, double theta)
{
   NEWMAT::Matrix Rot(3,3);
   NEWMAT::Matrix v(3,1);
   NEWMAT::Matrix omega(3,1);
   NEWMAT::IdentityMatrix I(3);
   NEWMAT::Matrix result(4,4);
   NEWMAT::Matrix Trans(3,1);
   double omegaMag;

   omega = GetAxis(xi);
   v = GetSpeed(xi);
   Rot = ExpRot(omega,theta);

   omegaMag = GetNorm(omega);

   if (omegaMag < EPS)
      Trans = v*theta;
   else
      Trans = (I - Rot)*cross(omega,v) + omega*omega.t()*v*theta;


   result = 0;

   result(1,1) = Rot(1,1);
   result(1,2) = Rot(1,2);
   result(1,3) = Rot(1,3);

   result(2,1) = Rot(2,1);
   result(2,2) = Rot(2,2);
   result(2,3) = Rot(2,3);

   result(3,1) = Rot(3,1);
   result(3,2) = Rot(3,2);
   result(3,3) = Rot(3,3);

   result(1,4) = Trans(1,1);
   result(2,4) = Trans(2,1);
   result(3,4) = Trans(3,1);
   result(4,4) = 1;

#ifdef DEBUG
   cout << "ExpTwist: v : " << endl <<  v << endl;
#endif

   return result;
};

NEWMAT::Matrix kinematics::GetAxis(NEWMAT::Matrix xi)
{
   NEWMAT::Matrix omega;
   omega = xi.SubMatrix(4,6,1,1);
   return omega;
}

NEWMAT::Matrix kinematics::GetSpeed(NEWMAT::Matrix xi)
{
   NEWMAT::Matrix v;
   v = xi.SubMatrix(1,3,1,1);
   return v;
}

NEWMAT::Matrix kinematics::Translate(double p[])
{
   NEWMAT::Matrix tM(4,4);

   tM(1,1) = 1;
   tM(1,2) = 0;
   tM(1,3) = 0;
   tM(1,4) = p[0];

   tM(2,1) = 0;
   tM(2,2) = 1;
   tM(2,3) = 0;
   tM(2,4) = p[1];

   tM(3,1) = 0;
   tM(3,2) = 0;
   tM(3,3) = 1;
   tM(3,4) = p[2];

   tM(4,1) = 0;
   tM(4,2) = 0;
   tM(4,3) = 0;
   tM(4,4) = 1;

   return tM;
};



void kinematics::MatrixToEuler(NEWMAT::Matrix matrix_in, double &roll, double &pitch, double &yaw, double &roll2, double &pitch2, double &yaw2)
{
  double* matrix_pointer = matrix_in.Store(); //get the pointer to the raw data
 
  if (fabs(matrix_pointer[8]) >= 1)  // Check that pitch is not at a singularity
    {
      pitch = - asin(matrix_pointer[8]);
      pitch2 = M_PI - pitch;

      roll = atan2(matrix_pointer[9]/cos(pitch), 
			     matrix_pointer[10]/cos(pitch));
      roll2 = atan2(matrix_pointer[9]/cos(pitch2), 
			      matrix_pointer[10]/cos(pitch2));
      
      yaw = atan2(matrix_pointer[4]/cos(pitch), 
			    matrix_pointer[1]/cos(pitch));
      yaw2 = atan2(matrix_pointer[4]/cos(pitch2), 
			     matrix_pointer[1]/cos(pitch2));
    }
  else
    {
      yaw = 0;
      yaw2 = 0;
      double delta = atan2(matrix_pointer[1],matrix_pointer[2]);      // From difference of angles formula
      if (matrix_pointer[8] > 0)  //gimbal locked up
	{
	  pitch = M_PI / 2.0;
	  pitch2 = M_PI / 2.0;
	  roll = pitch + delta;
	  roll2 = pitch + delta;
	}
      else // gimbal locked down
	{
	  pitch = -M_PI / 2.0;
	  pitch2 = -M_PI / 2.0;
	  roll = -pitch + delta;
	  roll2 = -pitch + delta;
	}
    }
  
}

NEWMAT::Matrix kinematics::Transform(double p[], double roll, double pitch, double yaw)
{
   NEWMAT::Matrix tM(4,4);
   NEWMAT::IdentityMatrix I(4);
   NEWMAT::Matrix xo(3,1),yo(3,1),zo(3,1);

   tM = I;
   xo = 0;
   yo = 0;
   zo = 0;

   xo(1,1) = 1;
   yo(2,1) = 1;
   zo(3,1) = 1;

   tM.SubMatrix(1,3,1,3) = ExpRot(zo,yaw)*ExpRot(yo,pitch)*ExpRot(xo,roll);

   tM(1,4) = p[0];
   tM(2,4) = p[1];
   tM(3,4) = p[2];

   return tM;
};


NEWMAT::Matrix kinematics::TwistVectorToMatrix(const NEWMAT::Matrix& xi)
{
   NEWMAT::Matrix xiHat(4,4);
   xiHat = 0;
   xiHat(1,1) = 0;
   xiHat(1,2) = -xi(6,1);
   xiHat(1,3) = -xi(5,1);

   xiHat(2,1) = xi(6,1);
   xiHat(2,2) = 0;
   xiHat(2,3) = -xi(4,1);

   xiHat(3,1) = -xi(5,1);
   xiHat(3,2) = -xi(4,1);
   xiHat(3,3) = 0;

   xiHat(1,4) = xi(1,1);
   xiHat(2,4) = xi(2,1);
   xiHat(3,4) = xi(3,1);
   xiHat(4,4) = 1;

   return xiHat;
};

NEWMAT::Matrix kinematics::TwistMatrixToVector(const NEWMAT::Matrix& xiHat)
{
   NEWMAT::Matrix xi(6,1);
#ifdef DEBUG
   cout << "TwistMatrixToVector::" << xiHat << endl;
#endif
   xi(1,1) = xiHat(1,4);
   xi(2,1) = xiHat(2,4);
   xi(3,1) = xiHat(3,4);

   xi(4,1) = -xiHat(2,3);
   xi(5,1) = -xiHat(3,1);
   xi(6,1) = -xiHat(1,2);

   return xi;
};

NEWMAT::Matrix kinematics::TransformTwist(const NEWMAT::Matrix& xi, const NEWMAT::Matrix&  g)
{
   NEWMAT::Matrix xi_new(6,1);
   NEWMAT::Matrix tM = kinematics::TwistVectorToMatrix(xi);

   xi_new = kinematics::TwistMatrixToVector(g*tM*g.i());

#ifdef DEBUG
   cout << "TransformTwist::xi" << xi << endl;
   cout << "TransformTwist::tM" << tM << endl;
   cout << "TransformTwist::xi_new" << xi_new << endl;
#endif

   return xi_new;
};


NEWMAT::Matrix kinematics::GetPosition(const NEWMAT::Matrix& p)
{
   NEWMAT::Matrix q(4,1);

   q.SubMatrix(1,3,1,1) = p.SubMatrix(1,3,4,4);
   q(4,1) = 1;
   return q;
};

NEWMAT::Matrix kinematics::GetRotationMatrix(const NEWMAT::Matrix& p)
{
   NEWMAT::Matrix q(3,3);

   q = p.SubMatrix(1,3,1,3);
   return q;
};

SerialRobot::SerialRobot(int nJoints)
{
   NEWMAT::IdentityMatrix I(4);
   jointCount = 0;
   homePosition = I;
   Initialize(nJoints);
};

SerialRobot::SerialRobot()
{
   NEWMAT::IdentityMatrix I(4);
   numberJoints = 0;
   jointCount = 0;
   homePosition = I;
};

void SerialRobot::Initialize(int nJoints)
{
   numberJoints = nJoints;
   joints = new Joint[numberJoints];
   min_joint_limits_.resize(numberJoints);
   max_joint_limits_.resize(numberJoints);
}

SerialRobot::~SerialRobot()
{
  delete [] joints;
//   joints = NULL;
};

void SerialRobot::SetHomePosition(NEWMAT::Matrix g)
{
   homePosition = g;
}

NEWMAT::Matrix SerialRobot::GetHomePosition()
{
   return homePosition;
}

void SerialRobot::AddJoint(NEWMAT::Matrix jin, double p[])
{
//   *(joints[jointCount]) = new NEWMAT::Matrix(6,1);

   joints[jointCount].twist = jin;
   joints[jointCount].jointId = jointCount;
   joints[jointCount].linkPose = Translate(p);
   min_joint_limits_[jointCount] = -M_PI;
   max_joint_limits_[jointCount] = M_PI;
#ifdef DEBUG
   cout << "AddJoint:: " << joints[jointCount].linkPose << endl;
   cout << "Link pose" << endl <<  joints[jointCount].linkPose << endl;
   cout << "twist " << endl <<  joints[jointCount].twist << endl;
#endif
   jointCount++;
};

void kinematics::cross(const double p1[], const double p2[], double p3[])
{
   p3[0] = p1[1]*p2[2]-p1[2]*p2[1];
   p3[1] = p1[2]*p2[0]-p1[0]*p2[2];
   p3[2] = p1[0]*p2[1]-p1[1]*p2[0];

#if DEBUG
   printf("p1: %f %f %f\n",p1[0],p1[1],p1[2]);
   printf("p2: %f %f %f\n",p2[0],p2[1],p2[2]);
   printf("p3: %f %f %f\n",p3[0],p3[1],p3[2]);
#endif
};

NEWMAT::Matrix kinematics::cross(NEWMAT::Matrix p1, NEWMAT::Matrix p2)
{
   NEWMAT::Matrix p3(3,1);
   p3(1,1) = p1(2,1)*p2(3,1)-p1(3,1)*p2(2,1);
   p3(2,1) = p1(3,1)*p2(1,1)-p1(1,1)*p2(3,1);
   p3(3,1) = p1(1,1)*p2(2,1)-p1(2,1)*p2(1,1);

   return p3;
};

void SerialRobot::AddJoint(double p[], double axis[], int jointType)
{
   NEWMAT::Matrix jj(6,1);
   switch(jointType)
   {
      case PRISMATIC:
         jj(1,1) = axis[0];
         jj(2,1) = axis[1];
         jj(3,1) = axis[2];
         jj(4,1) = 0;
         jj(5,1) = 0;
         jj(6,1) = 0;
#ifdef DEBUG
         printf("Joint type: PRISMATIC\n");
#endif
         break;
      case ROTARY:
         double q[3];
         cross(p,axis,q);
         jj(1,1) = q[0];
         jj(2,1) = q[1];
         jj(3,1) = q[2];
         jj(4,1) = axis[0];
         jj(5,1) = axis[1];
         jj(6,1) = axis[2];
#ifdef DEBUG
         printf("Joint type: ROTARY\n");
#endif
         break;
      default:
         break;
   };
   AddJoint(jj,p);
};



void SerialRobot::AddJoint(NEWMAT::Matrix p, NEWMAT::Matrix axis, string joint_type)
{
  NEWMAT::Matrix jj(6,1);
  double p1[3];

  p1[0] = p(1,1);
  p1[1] = p(2,1);
  p1[2] = p(3,1);

  if(joint_type == std::string("PRISMATIC"))
  {
    jj(1,1) = axis(1,1);
    jj(2,1) = axis(2,1);
    jj(3,1) = axis(3,1);
    jj(4,1) = 0;
    jj(5,1) = 0;
    jj(6,1) = 0;
#ifdef DEBUG
    printf("Joint type: PRISMATIC\n");
    cout << jj << endl;
#endif
  }
  else if(joint_type == std::string("ROTARY"))
  {
    NEWMAT::Matrix q;
    q = cross(p,axis);
    jj(1,1) = q(1,1);
    jj(2,1) = q(2,1);
    jj(3,1) = q(3,1);
    jj(4,1) = axis(1,1);
    jj(5,1) = axis(2,1);
    jj(6,1) = axis(3,1);
#ifdef DEBUG
    printf("Joint type: ROTARY\n");
    cout << jj << endl;
#endif
  }
  else
  {
    printf("SerialRobot::AddJoint - unrecognized joint type");
    return;
  };
  AddJoint(jj,p1);
};

void SerialRobot::SetJointLimits(const std::vector<double> &min_joint_limits, const std::vector<double> &max_joint_limits)
{
  if((int) min_joint_limits.size() != numberJoints || (int) max_joint_limits.size() != numberJoints)
  {
    printf("SerialRobot::SetJointLimits:: number of elements in min or max vectors do not match number of joint %d",jointCount);
  }
  for(int i=0; i < numberJoints; i++)
  {
    min_joint_limits_[i] = min_joint_limits[i];
    max_joint_limits_[i] = max_joint_limits[i];
  }
}

bool SerialRobot::CheckJointLimits(const std::vector<double> &joint_values)
{
  for(int i=0; i<numberJoints; i++)
  {
    if(!CheckJointLimits(joint_values[i],i))
      return false;
  }
  return true;
}

bool SerialRobot::CheckJointLimits(const double &joint_value, const int &joint_num)
{
 if(joint_value < min_joint_limits_[joint_num] || joint_value > max_joint_limits_[joint_num])
   return false;
 return true;
}

NEWMAT::Matrix SerialRobot::GetLinkPose(int id, double angles[])
{
   NEWMAT::Matrix returnPose;
   NEWMAT::IdentityMatrix I(4);
   if(id ==0)//ground link
      returnPose = I;
   else
      returnPose = ComputeFK(angles,id);

   return returnPose;
};

NEWMAT::Matrix SerialRobot::GetTwist(int id)
{
   return joints[id].twist;
};

NEWMAT::Matrix SerialRobot::GetJointExponential(int id, double theta)
{
#ifdef DEBUG
   cout << "id:" << id << endl << joints[id].twist << endl << theta << endl;
#endif
   return ExpTwist(joints[id].twist,theta);
};

NEWMAT::Matrix SerialRobot::GetJointAxisPoint(int id)
{
   return GetPosition(joints[id].linkPose);
};

NEWMAT::Matrix SerialRobot::GetJointAxis(int id)
{
   return GetAxis(joints[id].twist);
};


NEWMAT::Matrix SerialRobot::ComputeFK(double jointAngles[])
{
   NEWMAT::Matrix returnPose;
   returnPose = ComputeFK(jointAngles,numberJoints);

   return returnPose;
};
 
NEWMAT::Matrix SerialRobot::ComputeFK(double jointAngles[], int id)
{
   NEWMAT::Matrix returnPose(4,4);
   NEWMAT::IdentityMatrix I(4);

   returnPose = I;

   for(int ii = 0; ii < id; ii++)
   {
      returnPose = returnPose * ExpTwist(joints[ii].twist,jointAngles[ii]);
#ifdef DEBUG
      cout << "ii: " << ii << endl;
      cout << "ComputeFK::" << "Intermediate return pose" << endl << returnPose << endl;
      cout << "ComputeFK::" << "Twist exponential" << endl << ExpTwist(joints[ii].twist,jointAngles[ii]) << endl;
#endif
   }

#ifdef DEBUG
   cout << "ComputeFK::" << "link Pose " << endl << joints[id].linkPose << endl;
   cout << "returnPose::" << endl << returnPose << endl;
#endif
      returnPose = returnPose * joints[id-1].linkPose;

   return returnPose;
};

void SerialRobot::ComputeIK(NEWMAT::Matrix p, double jointAngles[]){};

NEWMAT::Matrix SerialRobot::ComputeManipulatorJacobian(double jointAngles[])
{
//
// Each column of the jacobian is the twist vector for that joint in the new configuration frame
// The resultant matrix represents the Manipulator jacobian (and not the traditional jacobian)
//
// A good reference for this is the book: "A Mathematical Introduction to Robotic Manipulation"
// by Richard Murray, Zexiang Li and Shankar Sastry
//
   NEWMAT::Matrix iP;
   NEWMAT::Matrix jacobian(6,numberJoints);
   NEWMAT::Matrix newTwist(6,1);
   NEWMAT::IdentityMatrix I(4);

   iP = I;

   for(int ii = 0; ii < numberJoints; ii++)
   {
      newTwist = TransformTwist(joints[ii].twist,iP);
      iP = iP*ExpTwist(joints[ii].twist,jointAngles[ii]);

      jacobian(1,ii+1) = newTwist(1,1);
      jacobian(2,ii+1) = newTwist(2,1);
      jacobian(3,ii+1) = newTwist(3,1);
      jacobian(4,ii+1) = newTwist(4,1);
      jacobian(5,ii+1) = newTwist(5,1);
      jacobian(6,ii+1) = newTwist(6,1);
   };
   return jacobian;
};

NEWMAT::Matrix SerialRobot::ComputeEndEffectorVelocity(double jointAngles[], double jointSpeeds[])
{
   NEWMAT::Matrix jacobian(6,numberJoints);
   NEWMAT::Matrix speed(numberJoints,1);
   NEWMAT::Matrix spatialVelocity(numberJoints,1);
   NEWMAT::Matrix spatialVelocityMatrix(4,4);
   NEWMAT::Matrix endEffectorVelocity(6,1);
   NEWMAT::Matrix endEffectorTVelocity(4,1);
   NEWMAT::Matrix endEffectorPosition(4,1);

   /* Initialize everything to zero */
   jacobian = 0;
   speed = 0;
   spatialVelocity = 0;
   spatialVelocityMatrix = 0;

   speed << jointSpeeds;

   endEffectorPosition = GetPosition(ComputeFK(jointAngles));
   jacobian = ComputeManipulatorJacobian(jointAngles);
   spatialVelocity = jacobian*speed;
   spatialVelocityMatrix = TwistVectorToMatrix(spatialVelocity);
   endEffectorTVelocity = spatialVelocityMatrix * endEffectorPosition;

   endEffectorVelocity(1,1) = endEffectorTVelocity(1,1);
   endEffectorVelocity(2,1) = endEffectorTVelocity(2,1);
   endEffectorVelocity(3,1) = endEffectorTVelocity(3,1);

   endEffectorVelocity(4,1) = spatialVelocity(4,1);
   endEffectorVelocity(5,1) = spatialVelocity(5,1);
   endEffectorVelocity(6,1) = spatialVelocity(6,1);

   return endEffectorVelocity;
};

double kinematics::SubProblem1(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix w)
{
   NEWMAT::Matrix u(3,1),v(3,1),up(3,1),vp(3,1);
   NEWMAT::Matrix p(3,1),q(3,1),r(3,1);

   NEWMAT::Matrix arg1, arg2;

   p = pin.SubMatrix(1,3,1,1);
   q = qin.SubMatrix(1,3,1,1);
   r = rin.SubMatrix(1,3,1,1);

#ifdef DEBUG
   PrintMatrix(p,"SubProblem1::p");
   PrintMatrix(q,"SubProblem1::q");
   PrintMatrix(r,"SubProblem1::r");
   PrintMatrix(w,"SubProblem1::w");
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
   cout << "SubProblem1::" << arg1(1,1) << "," << arg2(1,1) << endl;
   cout << endl << endl << endl;
#endif
   return atan2(arg1(1,1),arg2(1,1));
}

void kinematics::SubProblem2(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix omega1, NEWMAT::Matrix omega2, double theta[])
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

   theta[0] = SubProblem1(c,q,r,omega1);
   theta[1] = SubProblem1(p,c,r,omega2);

   z = alpha(1,1)*omega1 + beta(1,1)*omega2 - gamma*cross(omega1,omega2);
   c = z+r;

#ifdef DEBUG
   PrintMatrix(c,"SubProblem2::c");
   PrintMatrix(z,"SubProblem2::z");
#endif

   theta[2] = SubProblem1(c,q,r,omega1);
   theta[3] = SubProblem1(p,c,r,omega2);
}

