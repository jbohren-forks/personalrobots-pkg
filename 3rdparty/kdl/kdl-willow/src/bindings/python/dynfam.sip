

class InertiaMatrix{

%TypeHeaderCode
#include <inertiamatrix.hpp>
using namespace KDL;
%End


public:
	InertiaMatrix(double Ixx=0,double Iyy=0,double Izz=0,double Ixy=0,double Ixz=0,double Iyz=0);
	static InertiaMatrix Zero();
	Vector operator*(const Vector& v) const /Factory/;

};


