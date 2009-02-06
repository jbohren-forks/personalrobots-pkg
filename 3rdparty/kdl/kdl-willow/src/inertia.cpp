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

#include "inertia.hpp"

namespace KDL {

  Inertia::Inertia(double m, const Vector& cog, double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
    :cog_(cog),
     m_(m),
     I_(InertiaMatrix(Ixx,Iyy,Izz,Ixy,Ixz,Iyz))
  {}



  InertiaMatrix::InertiaMatrix(double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
  {
    data[0] = Ixx, 
    data[4] = Iyy;
    data[8] = Izz;
    data[1] = data[3] = Ixy;
    data[2] = data[6] = Ixz;
    data[5] = data[7] = Iyz;
  }
  
  InertiaMatrix::InertiaMatrix(const InertiaMatrix& inert)
  {
    for (unsigned int i=0; i<9; i++)
      data[i] = inert.data[i];
  }

  InertiaMatrix InertiaMatrix::operator=(const InertiaMatrix& inert)
  {
    return InertiaMatrix(data[0], data[4], data[8], data[1], data[2], data[5]);
  }

  Vector InertiaMatrix::operator*(const Vector& v) const {
    // Complexity : 9M+6A
    return Vector(data[0]*v(0) + data[3]*v(1) + data[6]*v(3),
                  data[1]*v(0) + data[4]*v(1) + data[7]*v(2),
                  data[2]*v(0) + data[5]*v(1) + data[8]*v(2));
  }
}

