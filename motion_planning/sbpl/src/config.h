#ifndef __CONFIG_H_
#define __CONFIG_H_


//if set, then heuristic is used if available
#define USE_HEUR 1

//memory debugging
#define MEM_CHECK 0

//regular debugging
#define DEBUG 0

#define ERR_EPS 0.0000001

//environment
#define ENVIRONMENT_TYPE NAV2D_ENV_TYPE
//possible types of environment
#define XXX_ENV_TYPE 0
#define NAV2D_ENV_TYPE 1

#define UNKNOWN_COST 1000000

#define MDP_ERRDELTA 0.01 


#endif

