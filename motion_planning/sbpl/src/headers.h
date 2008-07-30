#ifndef __HEADERS_H_
#define __HEADERS_H_

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>

using namespace std;

#include "config.h"


#if MEM_CHECK == 1
#define _CRTDBG_MAP_ALLOC 
#define CRTDBG_MAP_ALLOC
#endif

#include <stdlib.h> //have to go after the defines above

#if MEM_CHECH == 1
#include <crtdbg.h>
#endif

#include "utils/key.h"
#include "utils/mdpconfig.h"
#include "utils/mdp.h"
#include "planners/planner.h"
#include "discrete_space_information/environment.h"
#include "discrete_space_information/template/environment_XXX.h"
#include "discrete_space_information/nav2d/environment_nav2D.h"
#include "utils/list.h"
#include "utils/heap.h"
#include "planners/VI/viplanner.h"
#include "planners/ARAStar/araplanner.h"
#include "utils/utils.h"


#endif

