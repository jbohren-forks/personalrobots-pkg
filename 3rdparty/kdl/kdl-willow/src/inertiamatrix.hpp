
#ifndef KDLINERTIAMATRIX_HPP
#define KDLINERTIAMATRIX_HPP

#include "frames.hpp"

//------- class for only the Inertiamatrix --------

namespace KDL
{
	class InertiaMatrix{
		public:
			double data[9];

			InertiaMatrix(double Ixx=0,double Iyy=0,double Izz=0,double Ixy=0,double Ixz=0,double Iyz=0);

			static inline InertiaMatrix Zero(){
				return InertiaMatrix(0,0,0,0,0,0);
			};

			KDL::Vector operator*(const KDL::Vector& v) const;

			~InertiaMatrix();

	};

}

#endif

