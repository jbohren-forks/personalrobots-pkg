#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <kdl/joint.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace ublas = boost::numeric::ublas;

//------- inertia.hpp [sourced from KDL] -------------
/**
 * This class offers the inertia-structure of a body
 * */
class Inertia_advait{
	public:
		Inertia_advait(double m=0,double Ixx=0,double Iyy=0,double Izz=0,double Ixy=0,double Ixz=0,double Iyz=0);

		static inline Inertia_advait Zero(){
			return Inertia_advait(0,0,0,0,0,0,0);
		};

//		double getMass() const;
//		ublas::matrix<double> getInertiaMatrix() const;

		~Inertia_advait();

	public:
		double m;
		ublas::matrix<double> *I;
};


#endif

