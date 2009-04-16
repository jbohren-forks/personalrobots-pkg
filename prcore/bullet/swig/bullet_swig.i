 %module bullet
 %{
 /* Includes the header in the wrapper code */
#include "LinearMath/btQuadWord.h"
//#include "pybtVector3.h"
//#include "pybtQuaternion.h"
//#include "pybtMatrix3x3.h"
#include "pybtTransform.h"
 %}
 
%include "std_string.i"

%include "LinearMath/btScalar.h"
%include "LinearMath/btMinMax.h"
%include "LinearMath/btQuadWord.h"

%include "pybtVector3.h"
%include "pybtQuaternion.h"
%include "pybtMatrix3x3.h"
%include "pybtTransform.h"

%extend py::Quaternion{
  char * __str__()
  {
    static char temp[256];
    sprintf(temp, "[%f, %f, %f, %f]", self->getX(), self->getY(), self->getZ(),  self->getW());
    return &temp[0];
  };
  
  Quaternion  __rmul__(const Vector3& w)
{
  py::Quaternion& q = *self;
  return py::Quaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
};


 }

%extend py::Vector3 {
  char * __str__()
  {
    static char temp[256];
    sprintf(temp, "[%f, %f, %f]", self->getX(), self->getY(), self->getZ());
    return &temp[0];
  };
  

   Vector3 __rmul__(float s) 
  {
    return py::Vector3(self->m_floats[0] * s, self->m_floats[1] * s, self->m_floats[2] * s);
  };

  Vector3 __rdiv__(float s) 
  {
    btFullAssert(s != btScalar(0.0));
    return self->operator*(btScalar(1.0) / s);
  };

 }

%extend py::Matrix3x3
{
  char * __str__()
  {
    static char temp[1024];

    sprintf(temp, "[[%f, %f, %f]|\n|[%f, %f, %f]|\n|[%f, %f, %f]]",
            self->getRow(0).getX(), self->getRow(0).getY(), self->getRow(0).getZ(),
            self->getRow(1).getX(), self->getRow(1).getY(), self->getRow(1).getZ(),
            self->getRow(2).getX(), self->getRow(2).getY(), self->getRow(2).getZ()
            );
    return &temp[0];
  };


  py::Vector3  __rmul__(const py::Vector3& v)
  {
    return py::vecTimesMatrix(v, *self);
  }


  double getEulerZYXYaw(int solution_number = 1) 
  {
    double yaw, pitch, roll;
    self->getEulerZYX(yaw, pitch, roll, solution_number);
    return yaw;
  }

  double getEulerZYXPitch(int solution_number = 1) 
  {
    double yaw, pitch, roll;
    self->getEulerZYX(yaw, pitch, roll, solution_number);
    return pitch;
  }
  double getEulerZYXRoll(int solution_number = 1) 
  {
    double yaw, pitch, roll;
    self->getEulerZYX(yaw, pitch, roll, solution_number);
    return roll;
  }
}

%extend py::Transform
{
  char * __str__()
  {
    static char temp[1024];

    py::Vector3 orig = self->getOrigin();
    py::Matrix3x3 basis = self->getBasis();
    sprintf(temp, "Origin:\n[%f, %f, %f]\n\nBasis:\n[[%f, %f, %f]|\n|[%f, %f, %f]|\n|[%f, %f, %f]]",
            orig.getX(), orig.getY(), orig.getZ(),
            basis.getRow(0).getX(), basis.getRow(0).getY(), basis.getRow(0).getZ(),
            basis.getRow(1).getX(), basis.getRow(1).getY(), basis.getRow(1).getZ(),
            basis.getRow(2).getX(), basis.getRow(2).getY(), basis.getRow(2).getZ()
            );
    return &temp[0];
  };



}
