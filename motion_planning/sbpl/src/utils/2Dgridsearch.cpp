/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "../headers.h"




//---------------------initialization and destruction routines--------------------------------------------------------
SBPL2DGridSearch::SBPL2DGridSearch(int width_x, int height_y, float cellsize_m)
{
	iteration_ = 0;
    searchStates2D_ = NULL;

    width_ = width_x;
    height_ = height_y;
	cellSize_m_ = cellsize_m;
    
    startX_ = -1;
    startY_ = -1;
	goalX_ = -1;
	goalY_ = -1;

	largestcomputedoptf_ = 0;

	//compute dx, dy, dxintersects and dyintersects arrays
	computedxy();

	//allocate memory
	OPEN2D_ = new CHeap;
	if(!createSearchStates2D())
	{
		printf("ERROR: failed to create searchstatespace2D\n");
		exit(1);
	}
}
    
bool SBPL2DGridSearch::createSearchStates2D(void)
{
	int x,y;

    if(searchStates2D_ != NULL){
        printf("ERROR: We already have a non-NULL search states array\n");
        return false;
    }
    
    searchStates2D_ = new SBPL_2DGridSearchState* [width_];
    for(x = 0; x < width_; x++)
    {
        searchStates2D_[x] = new SBPL_2DGridSearchState[height_];
        for(y = 0; y < height_; y++)
        {
			searchStates2D_[x][y].iterationaccessed = iteration_;
	        searchStates2D_[x][y].x = x;
	        searchStates2D_[x][y].y = y;
			initializeSearchState2D(&searchStates2D_[x][y]);
        }
    }
    return true;
}

inline void SBPL2DGridSearch::initializeSearchState2D(SBPL_2DGridSearchState* state2D)
{
	state2D->g = INFINITECOST;
	state2D->heapindex = 0;
	state2D->iterationaccessed = iteration_;
}


void SBPL2DGridSearch::destroy()
{

	// destroy the OPEN list:
    if(OPEN2D_ != NULL){
        OPEN2D_->makeemptyheap();
        delete OPEN2D_;
        OPEN2D_ = NULL;
    }

    // destroy the 2D states:
    if(searchStates2D_ != NULL){
	    for(int x = 0; x < width_; x++)
		{
			delete [] searchStates2D_[x];
		}
		delete [] searchStates2D_;
        searchStates2D_ = NULL;
    }
}


void SBPL2DGridSearch::computedxy()
{

	//initialize some constants for 2D search
	dx_[0] = 1; dy_[0] = 1;	dxintersects_[0][0] = -1; dyintersects_[0][0] = -1; 
	dx_[1] = 1; dy_[1] = 0;	dxintersects_[1][0] = -1; dyintersects_[1][0] = -1;
	dx_[2] = 1; dy_[2] = -1;	dxintersects_[2][0] = -1; dyintersects_[2][0] = -1;
	dx_[3] = 0; dy_[3] = 1;	dxintersects_[3][0] = -1; dyintersects_[3][0] = -1;
	dx_[4] = 0; dy_[4] = -1;	dxintersects_[4][0] = -1; dyintersects_[4][0] = -1;
	dx_[5] = -1; dy_[5] = 1;	dxintersects_[5][0] = -1; dyintersects_[5][0] = -1;
	dx_[6] = -1; dy_[6] = 0;	dxintersects_[6][0] = -1; dyintersects_[6][0] = -1;
	dx_[7] = -1; dy_[7] = -1;	dxintersects_[7][0] = -1; dyintersects_[7][0] = -1;

    //Note: these actions have to be starting at 8 and through 15, since they 
    //get multiplied correspondingly in Dijkstra's search based on index
#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS == 16
    dx_[8] = 2; dy_[8] = 1;	dxintersects_[8][0] = 1; dyintersects_[8][0] = 0; dxintersects_[8][1] = 1; dyintersects_[8][1] = 1;
	dx_[9] = 1; dy_[9] = 2;	dxintersects_[9][0] = 0; dyintersects_[9][0] = 1; dxintersects_[9][1] = 1; dyintersects_[9][1] = 1;
	dx_[10] = -1; dy_[10] = 2;	dxintersects_[10][0] = 0; dyintersects_[10][0] = 1; dxintersects_[10][1] = -1; dyintersects_[10][1] = 1;
	dx_[11] = -2; dy_[11] = 1;	dxintersects_[11][0] = -1; dyintersects_[11][0] = 0; dxintersects_[11][1] = -1; dyintersects_[11][1] = 1;
	dx_[12] = -2; dy_[12] = -1;	dxintersects_[12][0] = -1; dyintersects_[12][0] = 0; dxintersects_[12][1] = -1; dyintersects_[12][1] = -1;
	dx_[13] = -1; dy_[13] = -2;	dxintersects_[13][0] = 0; dyintersects_[13][0] = -1; dxintersects_[13][1] = -1; dyintersects_[13][1] = -1;
	dx_[14] = 1; dy_[14] = -2;	dxintersects_[14][0] = 0; dyintersects_[14][0] = -1; dxintersects_[14][1] = 1; dyintersects_[14][1] = -1;
	dx_[15] = 2; dy_[15] = -1; dxintersects_[15][0] = 1; dyintersects_[15][0] = 0; dxintersects_[15][1] = 1; dyintersects_[15][1] = -1;
#endif		

	//compute distances
	for(int dind = 0; dind  < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dind++)
	{

		if(dx_[dind] != 0 && dy_[dind] != 0){
            if(dind <= 7)
                dxy_distance_mm_[dind] = (int)(cellSize_m_*1414);	//the cost of a diagonal move in millimeters
            else
                dxy_distance_mm_[dind] = (int)(cellSize_m_*2236);	//the cost of a move to 1,2 or 2,1 or so on in millimeters
		}
		else
			dxy_distance_mm_[dind] = (int)(cellSize_m_*1000);	//the cost of a horizontal move in millimeters
	}
}

//--------------------------------------------------------------------------------------------------------------------


//-----------------------------------------debugging functions--------------------------------------------------------------
void SBPL2DGridSearch::printvalues()
{



}
//--------------------------------------------------------------------------------------------------------------------


//-----------------------------------------main functions--------------------------------------------------------------
bool SBPL2DGridSearch::search(unsigned char** Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c, int goaly_c,  
							  SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{

    SBPL_2DGridSearchState *searchExpState = NULL;
    SBPL_2DGridSearchState *searchPredState = NULL;
    int numofExpands = 0;
	CKey key;

    //get the current time
	clock_t starttime = clock();
        
	//closed = 0
	iteration_++;

	//init start and goal coordinates
	startX_ = startx_c;
	startY_ = starty_c;
    goalX_ = goalx_c;
	goalY_ = goaly_c;

	//clear the heap
	OPEN2D_->makeemptyheap();

    //check the validity of start/goal
    if(!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c))
	{
		printf("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c, goalx_c, goaly_c);
		return false;
	}

    // initialize the start state
    searchExpState = &searchStates2D_[startX_][startY_];
    initializeSearchState2D(searchExpState);
	searchExpState->g = 0;
	key.key[0] = searchExpState->g + SBPL_2DGRIDSEARCH_HEUR2D(startX_,startY_);
	OPEN2D_->insertheap(searchExpState, key);
    
    //initialize the goal state
    initializeSearchState2D(&searchStates2D_[goalx_c][goaly_c]);
    SBPL_2DGridSearchState* search2DGoalState = &searchStates2D_[goalx_c][goaly_c];

	//set the termination condition
	float term_factor = 0.0;
	switch(termination_condition)
	{
	case SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND:
		term_factor = 1;
		break;
	case SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH:
		term_factor = (float)(1.0/1.2);
		break;
	case SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH:
		term_factor = 0.5;
		break;
	case SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH:
		term_factor = (float)(1.0/3.0);
		break;
	case SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS:
		term_factor = 0.0;
		break;
	default:
		printf("ERROR: incorrect termination factor for grid2Dsearch\n");
		term_factor = 0.0;
	};
	

    //the main repetition of expansions
    while(!OPEN2D_->emptyheap() &&  __min(INFINITECOST, search2DGoalState->g) > term_factor*OPEN2D_->getminkeyheap().key[0])
    {
        //get the next state for expansion
        searchExpState = (SBPL_2DGridSearchState*)OPEN2D_->deleteminheap();
        numofExpands++;

		int exp_x = searchExpState->x;
		int exp_y = searchExpState->y;

        //iterate over successors
		for(int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++)
		{
			int newx = exp_x + dx_[dir];
			int newy = exp_y + dy_[dir];		
						
			//make sure it is inside the map and has no obstacle
			if(!withinMap(newx, newy))
				continue;

			//compute the cost 
            int mapcost = __max(Grid2D[newx][newy], Grid2D[exp_x][exp_y]);

#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS > 8
            if(dir > 7){
                //check two more cells through which the action goes
                mapcost = __max(mapcost, Grid2D[exp_x + dxintersects_[dir][0]][exp_y + dyintersects_[dir][0]]);
                mapcost = __max(mapcost, Grid2D[exp_x + dxintersects_[dir][1]][exp_y + dyintersects_[dir][1]]);
            }
#endif

			if(mapcost >= obsthresh) //obstacle encountered
				continue; 
			int cost = (mapcost+1)*dxy_distance_mm_[dir];

			//get the predecessor
			searchPredState = &searchStates2D_[newx][newy];			

			//update predecessor if necessary
           if(searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g)
           {
				searchPredState->iterationaccessed = iteration_;
				searchPredState->g = __min(INFINITECOST, cost + searchExpState->g);
                key.key[0] = searchPredState->g + SBPL_2DGRIDSEARCH_HEUR2D(searchPredState->x, searchPredState->y);
				
				if(searchPredState->heapindex == 0)
					OPEN2D_->insertheap(searchPredState, key);
				else
					OPEN2D_->updateheap(searchPredState, key);
			}
        } //over successors
    }//while

	//set lower bounds for the remaining states
    if(!OPEN2D_->emptyheap()) 
		largestcomputedoptf_ = OPEN2D_->getminkeyheap().key[0];
	else
		largestcomputedoptf_ = INFINITECOST;


    printf("number of expansions during 2dgridsearch=%d time=%d msecs 2Dsolcost=%d largestoptfval=%d\n", 
		numofExpands, (int)(((clock()-starttime)/(double)CLOCKS_PER_SEC)*1000), searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_);
    
    //printHeuristicValues();


	return false;
}



//-----------------------------------------------------------------------------------------------------------------------


