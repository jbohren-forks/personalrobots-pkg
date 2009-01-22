// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "joint.hpp"
#include <iostream>

namespace KDL {

    // constructor for joint along x,y or z axes, at origin of reference frame
    Joint::Joint(const JointType& _type, const double& _scale, const double& _offset,
                 const double& _inertia, const double& _damping, const double& _stiffness):
        type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness)
    {
      assert(type != RotAxes && type != TransAxes);
    }

    // constructgor for joint along arbitrary axes, at arbitrary origin
    Joint::Joint(const Vector& _origin, const Vector& _axes, const JointType& _type, const double& _scale, const double& _offset,
	         const double& _inertia, const double& _damping, const double& _stiffness):
      origin(_origin), axes(_axes / _axes.Norm()), type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness)
    {
      assert(type == RotAxes || type == TransAxes);

      // pre-calculate stuff
      joint_pose.p = origin;
      joint_pose.M = Rotation::Identity();
      q_previous = 0;
      axes_sqr_0 = pow(axes(0),2);
      axes_sqr_1 = pow(axes(1),2);
      axes_sqr_2 = pow(axes(2),2);
    }

    Joint::~Joint()
    {
    }

    Frame Joint::pose(const double& q)const
    {

        switch(type){
	case RotAxes:{
	    q_previous = q;
	    Rotation& rot = joint_pose.M;

	    double cq = cos(scale*q+offset);
	    double cq_min = (1-cq);
	    double sq = sin(scale*q+offset);
	    
	    rot(0,0) = axes_sqr_0 + (axes_sqr_1+axes_sqr_2)*cq;
	    rot(1,1) = axes_sqr_1 + (axes_sqr_2+axes_sqr_0)*cq;
	    rot(2,2) = axes_sqr_2 + (axes_sqr_0+axes_sqr_1)*cq;
	    
	    double m_01_a = axes(0)*axes(1)*cq_min;
	    double m_01_b = axes(2)*sq;
	    rot(0,1) = m_01_a-m_01_b;
	    rot(1,0) = m_01_a+m_01_b;
	    
	    double m_02_a = axes(0)*axes(2)*cq_min;
	    double m_02_b = axes(1)*sq;
	    rot(0,2) = m_02_a+m_02_b;
	    rot(2,0) = m_02_a-m_02_b;
	    
	    double m_12_a = axes(1)*axes(2)*cq_min;
	    double m_12_b = axes(0)*sq;
	    rot(1,2) = m_12_a-m_12_b;
	    rot(2,1) = m_12_a+m_12_b;

	    return joint_pose;
	    break;}
        case RotX:
            return Frame(Rotation::RotX(scale*q+offset));
            break;
        case RotY:
            return  Frame(Rotation::RotY(scale*q+offset));
            break;
        case RotZ:
            return  Frame(Rotation::RotZ(scale*q+offset));
            break;
	case TransAxes:
	    return Frame(origin + (axes * (scale*q+offset)));
	    break;
        case TransX:
            return  Frame(Vector(scale*q+offset,0.0,0.0));
            break;
        case TransY:
            return Frame(Vector(0.0,scale*q+offset,0.0));
            break;
        case TransZ:
            return Frame(Vector(0.0,0.0,scale*q+offset));
            break;
        case None:
            return Frame::Identity();
            break;
        }
    }

    Twist Joint::twist(const double& qdot)const
    {
        switch(type){
	case RotAxes:
  	    return Twist(Vector(0,0,0), axes * (scale * qdot));
	    break;
        case RotX:
            return Twist(Vector(0.0,0.0,0.0),Vector(scale*qdot,0.0,0.0));
            break;
        case RotY:
            return Twist(Vector(0.0,0.0,0.0),Vector(0.0,scale*qdot,0.0));
            break;
        case RotZ:
            return Twist(Vector(0.0,0.0,0.0),Vector(0.0,0.0,scale*qdot));
            break;
	case TransAxes:
	    return Twist(axes * (scale * qdot), Vector(0,0,0));
	    break;
        case TransX:
            return Twist(Vector(scale*qdot,0.0,0.0),Vector(0.0,0.0,0.0));
            break;
        case TransY:
            return Twist(Vector(0.0,scale*qdot,0.0),Vector(0.0,0.0,0.0));
            break;
        case TransZ:
            return Twist(Vector(0.0,0.0,scale*qdot),Vector(0.0,0.0,0.0));
            break;
        case None:
            return Twist::Zero();
            break;
        }
    }

    Wrench Joint::wrench(const double& eff)const
    {
        switch(type){
        case RotX:
            return Wrench(Vector(0.0,0.0,0.0),Vector(scale*eff,0.0,0.0));
            break;
        case RotY:
            return Wrench(Vector(0.0,0.0,0.0),Vector(0.0,scale*eff,0.0));
            break;
        case RotZ:
            return Wrench(Vector(0.0,0.0,0.0),Vector(0.0,0.0,scale*eff));
            break;
        case TransX:
            return Wrench(Vector(scale*eff,0.0,0.0),Vector(0.0,0.0,0.0));
            break;
        case TransY:
            return Wrench(Vector(0.0,scale*eff,0.0),Vector(0.0,0.0,0.0));
            break;
        case TransZ:
            return Wrench(Vector(0.0,0.0,scale*eff),Vector(0.0,0.0,0.0));
            break;
        case None:
            return Wrench::Zero();
            break;
        }
    }
} // end of namespace KDL

