#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <kdl/joint.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "libKDL/kdl_kinematics.h"

namespace ublas = boost::numeric::ublas;

//------- class for only the Inertiamatrix --------
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


//------- Inertia class, copied from inertia.hpp and modified --------
class Inertia_advait{
	public:
		Inertia_advait(double m=0,double Ixx=0,double Iyy=0,double Izz=0,double Ixy=0,double Ixz=0,double Iyz=0);

		static inline Inertia_advait Zero(){
			return Inertia_advait(0,0,0,0,0,0,0);
		};

		~Inertia_advait();

	public:
		double m;
		InertiaMatrix I;
};




#endif

