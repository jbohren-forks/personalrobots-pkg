#include "matrix_utils.h"
#include <cmath>

vector<vector<double> > x_axis_rotation(double angle) 
{
	vector<vector<double> > R (3, vector<double>(3,0));
	R[0][0] = 1;
	R[1][1] = cos(angle);
	R[1][2] = sin(angle);
	R[2][1] = -sin(angle);
	R[2][2] = cos(angle);
	return R;
}

vector<vector<double> > y_axis_rotation(double angle) 
{
	vector<vector<double> > R (3, vector<double>(3,0));	
	R[0][0] = cos(angle);
	R[0][2] = -sin(angle);
	R[1][1] = 1;
	R[2][0] = sin(angle);
	R[2][2] = cos(angle);
	return R;
}

vector<vector<double> > z_axis_rotation(double angle) 
{
	vector<vector<double> > R (3, vector<double>(3,0));
	R[0][0] = cos(angle);
	R[0][1] = sin(angle);
	R[1][0] = -sin(angle);
	R[1][1] = cos(angle);
	R[2][2] = 1;
	return R;
}

vector<vector<double> > Rmultiply(vector<vector<double> > R1, vector<vector<double> > R2)
{
	vector<vector<double> > R1x2 (3, vector<double>(3,0));
	for (size_t i=0; i<R1.size(); i++) {
		for (size_t j=0; j<R1.size(); j++) {
			for (size_t k=0; k<R1.size(); k++) {
				R1x2[i][j] += R1[i][k]*R2[k][j];
			}
		}
	}
	return R1x2;
}

vector<double> apply_transformation(vector<double> v_in, vector<vector<double> > R, 
																		vector<double> T) 
{
	vector<double> v_out(3,0.0);
	for (size_t i=0; i<v_in.size(); i++) {
		for (size_t j=0; j<v_in.size(); j++) {
			v_out[i] += (R[i][j] * v_in[j]);
		}
		v_out[i] += T[i];
	}
	return v_out;
}

vector<double> apply_rotation(vector<double> v_in, vector<vector<double> > R)
{
	vector<double> v_out(3,0.0);
	for (size_t i=0; i<v_in.size(); i++) {
		for (size_t j=0; j<v_in.size(); j++) {
			v_out[i] += (R[i][j] * v_in[j]);
		}
	}
	return v_out;
}

double dot_product(vector<double> v1, vector<double> v2)
{
	double dp = 0;
	for (size_t i=0; i<v1.size(); i++) {
		dp += v1[i]*v2[i];
	}
	return dp;
}
