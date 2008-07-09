#ifndef _data_h_
#define _data_h_

struct cloud_point {
public:
	float x;
	float y;
	float z;
	float i;
	
cloud_point(float _x, float _y, float _z, float _i) : x(_x), y(_y), z(_z), i(_i) {}
cloud_point() : x(0), y(0), z(0), i(0) {}
};

#endif
