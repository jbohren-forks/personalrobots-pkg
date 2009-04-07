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

#include <sbpl/headers.h>
#include <sbpl_door_planner/environment_navxythetadoor.h>

int EnvironmentNAVXYTHETADOOR::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action)
{

    sbpl_2Dcell_t cell;
    vector<int> doorangleV, prevdoorangleV;
    vector<int> dooranglecostV, prevdooranglecostV;
    int i;

	
    if(!IsValidCell(SourceX, SourceY))
      {
	//          printf("\n\n\nReturning infinite cost\n\n\n");
	return INFINITECOST;
      }
    //compute world frame offset for the action
    double sourcex = DISCXY2CONT(SourceX, EnvNAVXYTHETALATCfg.cellsize_m);
    double sourcey = DISCXY2CONT(SourceY, EnvNAVXYTHETALATCfg.cellsize_m);

    //iterate over intermediate poses and compute the cost multiplier due to door
    int doorcostmultiplier = 0;
    for(i = 0; i < (int)action->intermptV.size(); i++) 
    {
        EnvNAVXYTHETALAT3Dpt_t point3D = action->intermptV.at(i);

        //get the pose translated into the world frame
        point3D.x = point3D.x + sourcex;
        point3D.y = point3D.y + sourcey;

        //get the door intervals together with the costs
        doorangleV.clear();
        dooranglecostV.clear();
//        printf("\n\n\n Getting action cost\n\n\n");
        GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV);
//        printf("Size angles: %d, costs: %d\n",doorangleV.size(),dooranglecostV.size());
        if(i > 0){
            //go over the previous interval and pick the common value with the smallest cost
            int mincost = INFINITECOST;
            int bestprevangle = -1;
            for(int cind = 0; cind < (int)doorangleV.size(); cind++){
                for(int pind = 0; pind < (int) prevdoorangleV.size(); pind++){
                    if(doorangleV[cind] == prevdoorangleV[pind] && __min(dooranglecostV[cind], prevdooranglecostV[pind]) < mincost){
                        mincost = __min(dooranglecostV[cind], prevdooranglecostV[pind]);
                        bestprevangle = prevdoorangleV[pind];
                    }
                }
            }
            //is the worst case cost
            doorcostmultiplier = __max(mincost, doorcostmultiplier);
        }

        //store the old interval
        prevdoorangleV = doorangleV;
        prevdooranglecostV = dooranglecostV;

    }    
//    printf("\n\n\n Skipping previous step\n\n\n");
    if(doorcostmultiplier >= INFINITECOST)
        return INFINITECOST;


	int currentmaxcost = 0;
	for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
	{
		//get the cell in the map
		cell = action->intersectingcellsV.at(i);
		cell.x = cell.x + SourceX;
		cell.y = cell.y + SourceY;
		
		//check validity
		if(!IsValidCell(cell.x, cell.y))
			return INFINITECOST;

		if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
			currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
	}

	//to ensure consistency of h2D:
	currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
		return INFINITECOST;
	currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);


	return action->cost*(currentmaxcost+1)*(doorcostmultiplier + 1); //use cell cost as multiplicative factor
 
}


void EnvironmentNAVXYTHETADOOR::GetValidDoorAngles(EnvNAVXYTHETALAT3Dpt_t worldrobotpose3D, vector<int>* doorangleV, 
                                                   vector<int>* dooranglecostV)
{

    //dummy test
    doorangleV->clear();
    dooranglecostV->clear();

/*    doorangleV->push_back(1);
    doorangleV->push_back(2);
    dooranglecostV->push_back(0);
    dooranglecostV->push_back(3);
*/
    robot_global_pose_[0] = worldrobotpose3D.x;
    robot_global_pose_[1] = worldrobotpose3D.y;
    robot_global_pose_[2] = worldrobotpose3D.theta;

    db_.getValidDoorAngles(footprint_, robot_global_pose_, door_global_pose_, robot_shoulder_position_, door_handle_pose_, door_length_, door_thickness_, pivot_length_, min_workspace_angle_, max_workspace_angle_, min_workspace_radius_, max_workspace_radius_, delta_angle_, *doorangleV, *dooranglecostV);

// No larger than 255 unsigned char
// Also put in an infinite cost
}


//setting desired door angles - see comments in environment_navxythetadoor.h file
void EnvironmentNAVXYTHETADOOR::SetDesiredDoorAngles(vector<int> desired_door_anglesV)
{

	//set the vector
	this->desired_door_anglesV.clear();
	this->desired_door_anglesV = desired_door_anglesV;

	printf("desired door angles are set to %d values\n", this->desired_door_anglesV.size());
}


int EnvironmentNAVXYTHETADOOR::MinCostDesiredDoorAngle(int x, int y, int theta)
{

  vector<int> doorangleV;
  vector<int> dooranglecostV;

  //special case of no desired door angles
  if(desired_door_anglesV.size() == 0)
    return INFINITECOST;

  //get the world 3d robot pose
  EnvNAVXYTHETALAT3Dpt_t point3D;
  point3D.x = DISCXY2CONT(x, EnvNAVXYTHETALATCfg.cellsize_m);
  point3D.y = DISCXY2CONT(y, EnvNAVXYTHETALATCfg.cellsize_m);
  point3D.theta = DiscTheta2Cont(theta, NAVXYTHETALAT_THETADIRS);

  //get the door intervals together with the costs
  doorangleV.clear();
  dooranglecostV.clear();
  GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV);

  //go over the interval and pick the desired angle with the smallest cost
  int mincost = INFINITECOST;
  int bestangle = -1;
  for(int cind = 0; cind < (int)doorangleV.size(); cind++){
    for(int dind = 0; dind < (int) desired_door_anglesV.size(); dind++){
      if(doorangleV[cind] == desired_door_anglesV[dind] && dooranglecostV[cind] < mincost){
	mincost = dooranglecostV[cind];
	bestangle = desired_door_anglesV[dind];
      }
    }
  }

  return mincost;

}



//overwrites the parent navxythetalat class to return a goal whenever a state has the desired angle door, independently of the robot pose
void EnvironmentNAVXYTHETADOOR::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
    if(actionV != NULL)
      {
	actionV->clear();
	actionV->reserve(EnvNAVXYTHETALATCfg.actionwidth);
      }
    
    //goal state should be absorbing
    if(SourceStateID == EnvNAVXYTHETALAT.goalstateid)
      return;
    
    //get X, Y for the state
    EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[SourceStateID];
    
    //iterate through actions
    for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
      {
	EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[HashEntry->Theta][aind];
	int newX = HashEntry->X + nav3daction->dX;
	int newY = HashEntry->Y + nav3daction->dY;
	int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETALAT_THETADIRS);	
	
	//skip the invalid cells
	if(!IsValidCell(newX, newY)) 
	  continue;
	
	//get cost
	int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
	if(cost >= INFINITECOST)
	  continue;
	
	//check that the out state is not goal in term of desired door_angles
	int mincostofdesireddoorangle = MinCostDesiredDoorAngle(newX,newY,newTheta);
	if (mincostofdesireddoorangle < INFINITECOST)
	  {
	    //insert the goal (NOTE: we ignore the final door cost)
	    SuccIDV->push_back(EnvNAVXYTHETALAT.goalstateid);
	  }
	else{		
	  EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
	  if((OutHashEntry = GetHashEntry(newX, newY, newTheta)) == NULL)
	    {
	      //have to create a new entry
	      OutHashEntry = CreateNewHashEntry(newX, newY, newTheta);
	    }
	  SuccIDV->push_back(OutHashEntry->stateID);
	}
	
	CostV->push_back(cost);
	if(actionV != NULL)
	  actionV->push_back(nav3daction);
      }
    
#if TIME_DEBUG
    time_getsuccs += clock()-currenttime;
#endif

}

