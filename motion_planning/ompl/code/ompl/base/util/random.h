#ifndef OMPL_RANDOM_
#define OMPL_RANDOM_

namespace ompl
{
    namespace random_utils
    {
	
	/** Random number generator state */
	struct rngState
	{
	    unsigned int seed;
	    struct
	    {
		double last;
		bool   valid;
	    } gaussian;
	};
	
	/** Initialize random number generator */
	void random_init(void);
	void random_init(rngState *state);
	
	/** Uniform random number generator */	
	double uniform(double lower_bound, double upper_bound);
	double uniform(rngState *state, double lower_bound, double upper_bound);
	
	int    uniformInt(int lower_bound, int upper_bound);
	int    uniformInt(rngState *state, int lower_bound, int upper_bound);
	
	/** Gaussian random number generator */	
	double gaussian(double mean, double stddev);
	double gaussian(rngState *state, double mean, double stddev);
	
	double bounded_gaussian(double mean, double stddev, double max_stddev);
	double bounded_gaussian(rngState *state, double mean, double stddev, 
				double max_stddev);
    }

}

#endif
    
    
