
#include "inertiamatrix.hpp"

namespace KDL
{
	InertiaMatrix::InertiaMatrix(double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
	{
		data[0] = Ixx, data[4] = Iyy, data[8] = Izz;
		data[1] = data[3] = Ixy;
		data[2] = data[6] = Ixz;
		data[5] = data[7] = Iyz;
	}

	InertiaMatrix::~InertiaMatrix()
	{
	}

	Vector InertiaMatrix::operator*(const Vector& v) const {
		// Complexity : 9M+6A
		return Vector(
				data[0]*v.data[0] + data[1]*v.data[1] + data[2]*v.data[2],
				data[3]*v.data[0] + data[4]*v.data[1] + data[5]*v.data[2],
				data[6]*v.data[0] + data[7]*v.data[1] + data[8]*v.data[2]
				);
	}

}

