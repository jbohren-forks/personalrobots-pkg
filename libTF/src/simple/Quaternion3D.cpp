// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
#include "libTF/Quaternion3D.h"

Euler3D::Euler3D(double _x, double _y, double _z, double _yaw, double _pitch, double _roll) :
  x(_x),y(_y),z(_z),yaw(_yaw),pitch(_pitch),roll(_roll)
{
  return;
};


Quaternion3D::Quaternion3D():
  max_storage_time(MAX_STORAGE_TIME),
  first(NULL),
  last(NULL)
{
  
  pthread_mutex_init( &linked_list_mutex, NULL);
  //fixme Normalize();
  return;
};

void Quaternion3D::fromQuaternion(double _xt, double _yt, double _zt, double _xr, double _yr, double _zr, double _w, unsigned long long time)
{
 Quaternion3DStorage temp;
 temp.xt = _xt; temp.yt = _yt; temp.zt = _zt; temp.xr = _xr; temp.yr = _yr; temp.zr = _zr; temp.w = _w; temp.time = time;

 add_value(temp);
 
} ;


void Quaternion3D::fromMatrix(const NEWMAT::Matrix& matIn, unsigned long long time)
{
  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html
  Quaternion3DStorage temp;
  temp.time = time;  

  const double * mat = matIn.Store();
  //Get the translations
  temp.xt = mat[3];
  temp.yt = mat[7];
  temp.zt = mat[11];

  //TODO ASSERT others are zero and one as they should be


  double T  = 1 + mat[0] + mat[5] + mat[10];


  //  If the trace of the matrix is greater than zero, then
  //  perform an "instant" calculation.
  //	      Important note wrt. rouning errors:

  if ( T > 0.00000001 ) //to avoid large distortions!
    {
      double S = sqrt(T) * 2;
      temp.xr = ( mat[9] - mat[6] ) / S;
      temp.yr = ( mat[2] - mat[8] ) / S;
      temp.zr = ( mat[4] - mat[1] ) / S;
      temp.w = 0.25 * S;
    }
  //If the trace of the matrix is equal to zero then identify
  // which major diagonal element has the greatest value.
  //  Depending on this, calculate the following:
  else if ( mat[0] > mat[5] && mat[0] > mat[10] ) {// Column 0: 
        double S  = sqrt( 1.0 + mat[0] - mat[5] - mat[10] ) * 2;
        temp.xr = 0.25 * S;
        temp.yr = (mat[1] + mat[4] ) / S;
        temp.zr = (mat[8] + mat[2] ) / S;
        temp.w = (mat[6] - mat[9] ) / S;
      } else if ( mat[5] > mat[10] ) {// Column 1: 
        double S  = sqrt( 1.0 + mat[5] - mat[0] - mat[10] ) * 2;
        temp.xr = (mat[1] + mat[4] ) / S;
        temp.yr = 0.25 * S;
        temp.zr = (mat[6] + mat[9] ) / S;
        temp.w = (mat[8] - mat[2] ) / S;
      } else {// Column 2:
        double S  = sqrt( 1.0 + mat[10] - mat[0] - mat[5] ) * 2;
        temp.xr = (mat[8] + mat[2] ) / S;
        temp.yr = (mat[6] + mat[9] ) / S;
        temp.zr = 0.25 * S;
        temp.w = (mat[1] - mat[4] ) / S;
      }

      add_value(temp);
};

void Quaternion3D::fromEuler(double _x, double _y, double _z, double _yaw, double _pitch, double _roll, unsigned long long time)
{
  fromMatrix(matrixFromEuler(_x,_y,_z,_yaw,_pitch,_roll),time);
};

void Quaternion3D::fromDH(double length, double alpha, double offset, double theta, unsigned long long time)
{
  fromMatrix(matrixFromDH(length, alpha, offset, theta),time);
};


NEWMAT::Matrix Quaternion3D::matrixFromEuler(double ax,
					     double ay, double az, double yaw,
					     double pitch, double roll)
{
  NEWMAT::Matrix matrix(4,4);
  double ca = cos(yaw);
  double sa = sin(yaw);
  double cb = cos(pitch);
  double sb = sin(pitch);
  double cg = cos(roll);
  double sg = sin(roll);
  double sbsg = sb*sg;
  double sbcg = sb*cg;


  double* matrix_pointer = matrix.Store();

  matrix_pointer[0] =  ca*cb;
  matrix_pointer[1] = (ca*sbsg)-(sa*cg);
  matrix_pointer[2] = (ca*sbcg)+(sa*sg);
  matrix_pointer[3] = ax;
  matrix_pointer[4] = sa*cb;
  matrix_pointer[5] = (sa*sbsg)+(ca*cg);
  matrix_pointer[6] = (sa*sbcg)-(ca*sg);
  matrix_pointer[7] = ay;
  matrix_pointer[8] = -sb;
  matrix_pointer[9] = cb*sg;
  matrix_pointer[10] = cb*cg;
  matrix_pointer[11] = az;
  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return matrix;
};


// Math from http://en.wikipedia.org/wiki/Robotics_conventions
NEWMAT::Matrix Quaternion3D::matrixFromDH(double length,
					  double alpha, double offset, double theta)
{
  NEWMAT::Matrix matrix(4,4);
  
  double ca = cos(alpha);
  double sa = sin(alpha);
  double ct = cos(theta);
  double st = sin(theta);
  
  double* matrix_pointer = matrix.Store();
  
  matrix_pointer[0] =  ct;
  matrix_pointer[1] = -st*ca;
  matrix_pointer[2] = st*sa;
  matrix_pointer[3] = length * ct;
  matrix_pointer[4] = st;
  matrix_pointer[5] = ct*ca;
  matrix_pointer[6] = -ct*sa;
  matrix_pointer[7] = length*st;
  matrix_pointer[8] = 0;
  matrix_pointer[9] = sa;
  matrix_pointer[10] = ca;
  matrix_pointer[11] = offset;
  matrix_pointer[12] = 0.0;
  matrix_pointer[13] = 0.0;
  matrix_pointer[14] = 0.0;
  matrix_pointer[15] = 1.0;

  return matrix;
};


Quaternion3D::Quaternion3DStorage& Quaternion3D::Quaternion3DStorage::operator=(const Quaternion3DStorage & input)
{
  xt = input.xt;
  yt = input.yt;
  zt = input.zt;
  xr = input.xr;
  yr = input.yr;
  zr = input.zr;
  w  = input.w ;
  time = input.time;

  return *this;
};



void Quaternion3D::Quaternion3DStorage::Normalize()
{
  double mag = getMagnitude();
  xr /= mag;
  yr /= mag;
  zr /= mag;
  w /= mag;
};

double Quaternion3D::Quaternion3DStorage::getMagnitude()
{
  return sqrt(xr*xr + yr*yr + zr*zr + w*w);
};

//Note global scope
std::ostream & operator<<(std::ostream& mystream, const Quaternion3D::Quaternion3DStorage& storage)
{

  mystream << "Storage: " << storage.xt <<", " << storage.yt <<", " << storage.zt<<", " << storage.xr<<", " << storage.yr <<", " << storage.zr <<", " << storage.w<<std::endl; 
  return mystream;
};

Quaternion3D::Quaternion3DStorage Quaternion3D::getQuaternion(unsigned long long time)
{
  Quaternion3DStorage temp;
  long long diff_time; //todo Find a way to use this offset. pass storage by reference and return diff_time??
  getValue(temp, time, diff_time);
  return temp;
};


NEWMAT::Matrix Quaternion3D::getMatrix(unsigned long long time)
{
  Quaternion3DStorage temp;
  long long diff_time;
  getValue(temp, time, diff_time);

  //print Storage:
  //  std::cout << temp;
 
  return temp.asMatrix();
}  

NEWMAT::Matrix Quaternion3D::getInverseMatrix(unsigned long long time)
{
  return getMatrix(time).i();

};

NEWMAT::Matrix Quaternion3D::Quaternion3DStorage::asMatrix()
{
  
  NEWMAT::Matrix outMat(4,4);
  
  double * mat = outMat.Store();

  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html
  double xx      = xr * xr;
  double xy      = xr * yr;
  double xz      = xr * zr;
  double xw      = xr * w;
  double yy      = yr * yr;
  double yz      = yr * zr;
  double yw      = yr * w;
  double zz      = zr * zr;
  double zw      = zr * w;
  mat[0]  = 1 - 2 * ( yy + zz );
  mat[4]  =     2 * ( xy - zw );
  mat[8]  =     2 * ( xz + yw );
  mat[1]  =     2 * ( xy + zw );
  mat[5]  = 1 - 2 * ( xx + zz );
  mat[9]  =     2 * ( yz - xw );
  mat[2]  =     2 * ( xz - yw );
  mat[6]  =     2 * ( yz + xw );
  mat[10] = 1 - 2 * ( xx + yy );
  mat[12]  = mat[13] = mat[14] = 0;
  mat[3] = xt;
  mat[7] = yt;
  mat[11] = zt;
  mat[15] = 1;
    

  return outMat;
};


void Quaternion3D::printMatrix(unsigned long long time)
{
  std::cout << getMatrix(time);
};

void Quaternion3D::printStorage(const Quaternion3DStorage& storage)
{
  std::cout << storage;
};


bool Quaternion3D::getValue(Quaternion3DStorage& buff, unsigned long long time, long long  &time_diff)
{
  Quaternion3DStorage p_temp_1;
  Quaternion3DStorage p_temp_2;
  //  long long temp_time;
  int num_nodes;

  bool retval = false;

  pthread_mutex_lock(&linked_list_mutex);
  num_nodes = findClosest(p_temp_1,p_temp_2, time, time_diff);

  if (num_nodes == 0)
    retval= false;
  else if (num_nodes == 1)
    {
      memcpy(&buff, &p_temp_1, sizeof(Quaternion3DStorage));
      retval = true;  
    }
  else
    {
      interpolate(p_temp_1, p_temp_2, time, buff); 
      retval = true;  
    }
  
  pthread_mutex_unlock(&linked_list_mutex);


  return retval;

};

void Quaternion3D::add_value(const Quaternion3DStorage &dataIn)
{
  pthread_mutex_lock(&linked_list_mutex);
  insertNode(dataIn);
  pruneList();
  pthread_mutex_unlock(&linked_list_mutex);
};


void Quaternion3D::insertNode(const Quaternion3DStorage & new_val)
{

  data_LL* p_current;
  data_LL* p_old;

  //  cout << "Inserting Node" << endl;

  //Base case empty list
  if (first == NULL)
    {
      //cout << "Base case" << endl;
      first = new data_LL;
      first->data = new_val;
      first->next = NULL;
      first->previous = NULL;
      last = first;
    }
  else 
    {
      //Increment through until at the end of the list or in the right spot
      p_current = first;
      while (p_current != NULL && first->data.time > new_val.time)
	{
	  //cout << "passed beyond " << p_current->data.time << endl;
	  p_current = p_current->next;
	}
      
      //THis means we hit the end of the list so just append the node
      if (p_current == NULL)
	{
	  //cout << "Appending node to the end" << endl;
	  p_current = new data_LL;
	  p_current->data = new_val;
	  p_current->previous = last;
	  p_current->next = NULL;

	  last = p_current;
	}
      else
	{
	  
	  //  cout << "Found a place to put data into the list" << endl;
	  
	  // Insert the new node
	  // Record where the old first node was
	  p_old = p_current;
	  //Fill in the new node
	  p_current = new data_LL;
	  p_current->data = new_val;
	  p_current->next = p_old;
	  p_current->previous = p_old->previous;
	  
	  //point the old to the new 
	  p_old->previous = p_current;

	  //If at the top of the list make sure we're not 
	  if (p_current->previous == NULL)
	    first = p_current;
	}



    }
};

void Quaternion3D::pruneList()
{
  data_LL* p_current = last;

  //  cout << "Pruning List" << endl;

  //Empty Set
  if (last == NULL) return;

  unsigned long long current_time = first->data.time;;


  //While time stamps too old
  while (p_current->data.time + max_storage_time < current_time)
    {
      //      cout << "Age of node " << (double)(-p_current->data.time + current_time)/1000000.0 << endl;
     // Make sure that there's at least two elements in the list
      if (p_current->previous != NULL)
	{
	  if (p_current->previous->previous != NULL)
	    {
	      // Remove the last node
	      p_current->previous->next = NULL;
	      last = p_current->previous;
	      delete p_current;
	      p_current = last;
	      //	      cout << " Pruning Node" << endl;
	    }
	  else 
	    break;
	}
      else 
	break;

    }
  
};



int Quaternion3D::findClosest(Quaternion3DStorage& one, Quaternion3DStorage& two, const unsigned long long target_time, long long &time_diff)
{

  data_LL* p_current = first;


  // Base case no list
  if (first == NULL)
    {
      return 0;
    }
  
  //Case one element list
  else if (first->next == NULL)
    {
      one = first->data;
      time_diff = target_time - first->data.time;
      return 1;
    }
  
  else
    {
      //Two or more elements
      //Find the one that just exceeds the time or hits the end
      //and then take the previous one
      p_current = first->next; //Start on the 2nd element so if we fail we fall back to the first one
      while (p_current->next != NULL && p_current->data.time > target_time)
	{
	  p_current = p_current->next;
	}
      
      one = p_current->data;
      two = p_current->previous->data;
      
      //FIXME this should be the min distance not just one random one.
      time_diff = target_time - two.time; 
      return 2;
    }
};

// Quaternion slerp algorithm from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
void Quaternion3D::interpolate(const Quaternion3DStorage &one, const Quaternion3DStorage &two,unsigned long long target_time, Quaternion3DStorage& output)
{
  output.time = target_time;

  // Calculate the ration of time betwen target and the two end points.
  long long total_diff = two.time - one.time;
  long long target_diff = target_time - one.time;

  //Check for zero distance case and just return
  if (abs(total_diff) < MIN_INTERPOLATION_DISTANCE || abs(target_diff) < MIN_INTERPOLATION_DISTANCE)
    {
      output = one;
      return;
    } 

  double t = (double)target_diff / (double) total_diff; //ratio between first and 2nd position interms of time

  // Interpolate the translation
  output.xt = one.xt + t * (two.xt - one.xt);  
  output.yt = one.yt + t * (two.yt - one.yt);  
  output.zt = one.zt + t * (two.zt - one.zt);  

  // Calculate angle between them.
  double cosHalfTheta = one.w * two.w + one.xr * two.xr + one.yr * two.yr + one.zr * two.zr;
  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (cosHalfTheta >= 1.0 || cosHalfTheta <= -1.0){
    output.w = one.w;output.xr = one.xr;output.yr = one.yr;output.zr = one.zr;
    return;
  }
  // Calculate temporary values.
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to qa or qb
  if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
    output.w = (one.w * 0.5 + two.w * 0.5);
    output.xr = (one.xr * 0.5 + two.xr * 0.5);
    output.yr = (one.yr * 0.5 + two.yr * 0.5);
    output.zr = (one.zr * 0.5 + two.zr * 0.5);
    return;
  }

  double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(t * halfTheta) / sinHalfTheta; 
  //calculate Quaternion.
  output.w = (one.w * ratioA + two.w * ratioB);
  output.xr = (one.xr * ratioA + two.xr * ratioB);
  output.yr = (one.yr * ratioA + two.yr * ratioB);
  output.zr = (one.zr * ratioA + two.zr * ratioB);
  return;
};

double Quaternion3D::interpolateDouble(const double first, const unsigned long long first_time, const double second, const unsigned long long second_time, const unsigned long long target_time)
{
  if ( first_time == second_time ) {
    return first;
  } else {
    return first + (second-first)* (double)((target_time - first_time)/(second_time - first_time));
  }
};
