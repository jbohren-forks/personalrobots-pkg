
#ifndef _UTILS_H_
#define _UTILS_H_

#include <vector>
#include <stdlib.h>
#include <time.h>
namespace librf {

	void random_sample(int n, int K, vector<int>*v) 
	{
		v->resize(0);
		if (K < n) 
		{
			v->reserve(K);
			for (int i = K; i > 0; --i) 
			{
				int x = (int)rand() % n;
				v->push_back(x);
			}
		}
		else 
		{
			for (int i =0; i < n; i++) {
				v->push_back(i);
			}
		}
	}
} // namespace
#endif
