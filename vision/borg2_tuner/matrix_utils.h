#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H

#include <vector>

using namespace std;

vector<vector<double> > x_axis_rotation(double angle);
vector<vector<double> > y_axis_rotation(double angle);
vector<vector<double> > z_axis_rotation(double angle) ;
vector<vector<double> > Rmultiply(vector<vector<double> > R1, 
																	vector<vector<double> > R2);
vector<double> apply_transformation(vector<double> v_in, vector<vector<double> > R, 
																		vector<double> T);
vector<double> apply_rotation(vector<double> v_in, vector<vector<double> > R);
double dot_product(vector<double> v1, vector<double> v2);

#endif
