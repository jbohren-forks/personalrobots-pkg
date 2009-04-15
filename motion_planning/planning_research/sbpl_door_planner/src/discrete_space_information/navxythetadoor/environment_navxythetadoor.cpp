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

template EnvironmentNAVXYTHETALAT<EnvNAVXYTHETADOORHashEntry_t>; 


int EnvironmentNAVXYTHETADOOR::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action)
{

    sbpl_2Dcell_t cell;
    vector<int> doorangleV, prevdoorangleV;
    vector<int> dooranglecostV, prevdooranglecostV;
    int i;

	
    if(!IsValidCell(SourceX, SourceY))
      {
	printf("\n\n\nReturning infinite cost\n\n\n");
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

    robot_msgs::Point32 robot_global_pose;
    double robot_global_yaw;

    robot_global_pose.x = worldrobotpose3D.x;
    robot_global_pose.y = worldrobotpose3D.y;
    robot_global_yaw = worldrobotpose3D.theta;

    db_.getValidDoorAngles(robot_global_pose,robot_global_yaw,*doorangleV,*dooranglecostV);

//    for(int i=0; i<doorangleV->size(); i++)
//    {
//      printf("Valid angle [%d]: %d, %d\n",i,doorangleV->at(i),dooranglecostV->at(i));
//    }
// No larger than 255 unsigned char
// Also put in an infinite cost
}

//setting desired door angles - see comments in environment_navxythetadoor.h file
void EnvironmentNAVXYTHETADOOR::SetDesiredDoorAngles(vector<int> desired_door_anglesV)
{
  std::vector<int> desired_door_angles_local;
  desired_door_angles_local.resize(desired_door_anglesV.size());
  //set the vector
  db_.getDesiredDoorAngles(desired_door_anglesV, desired_door_angles_local);

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


bool EnvironmentNAVXYTHETADOOR::GetMinCostDoorAngle(double x, double y, double theta, double &angle, double &door_angle_cost)
{
  vector<int> doorangleV;
  vector<int> dooranglecostV;

  //get the world 3d robot pose
  EnvNAVXYTHETALAT3Dpt_t point3D;
  point3D.x = x;
  point3D.y = y;
  point3D.theta = theta;

  //get the door intervals together with the costs
  doorangleV.clear();
  dooranglecostV.clear();
  GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV);

  //go over the interval and pick the angle with the smallest cost
  int mincost = INFINITECOST;
  int bestangle = -INFINITECOST;
  for(int cind = 0; cind < (int)doorangleV.size(); cind++)
  {
//    printf("angles[%d]: %d %d\n",cind,doorangleV[cind],dooranglecostV[cind]);
      if(dooranglecostV[cind] < mincost)
      {
	mincost = dooranglecostV[cind];
	bestangle = doorangleV[cind];
      }
  }

  if(bestangle != -INFINITECOST)
  {    
    angle = angles::normalize_angle((double) angles::from_degrees(bestangle) + db_.door_frame_global_yaw_);
    door_angle_cost = mincost;
//    printf("bestangle: %d\n",bestangle);
    return true;
  }
  return false;
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

void EnvironmentNAVXYTHETADOOR::setDoorDiscretizationAngle(const double &door_angle_discretization_interval)
{
  db_.door_angle_discretization_interval_ = door_angle_discretization_interval;
}

void EnvironmentNAVXYTHETADOOR::setDoorProperties(const robot_msgs::Door &door, 
                                                  double door_thickness)
{

  db_.door_thickness_ = door_thickness;

  double hinge_global_x = door.frame_p1.x;
  double hinge_global_y = door.frame_p1.y;
  double hinge_global_z = door.frame_p1.z;

  double edge_global_x = door.frame_p2.x;
  double edge_global_y = door.frame_p2.y;
  double edge_global_z = door.frame_p2.z;

  if(door.hinge == 1)
  {
    hinge_global_x = door.frame_p2.x;
    hinge_global_y = door.frame_p2.y;
    hinge_global_z = door.frame_p2.z;

    edge_global_x = door.frame_p1.x;
    edge_global_y = door.frame_p1.y;
    edge_global_z = door.frame_p1.z;
  }

  db_.door_frame_global_yaw_ = atan2(edge_global_y-hinge_global_y,edge_global_x-hinge_global_x);
  db_.door_length_ = sqrt(pow(edge_global_y-hinge_global_y,2) + pow(edge_global_x-hinge_global_x,2));

  double sth = sin(db_.door_frame_global_yaw_);
  double cth = cos(db_.door_frame_global_yaw_);

  db_.door_frame_global_position_.x = hinge_global_x;
  db_.door_frame_global_position_.y = hinge_global_y;

  // Need to transform handle pose from global frame to local door frame - TODO handle pose in Door message should already be in local frame
  db_.door_handle_position_.x = door.handle.x*cth+door.handle.y*sth-hinge_global_x*cth-hinge_global_y*sth;
  db_.door_handle_position_.y = -door.handle.x*sth+door.handle.y*cth+hinge_global_x*sth-hinge_global_y*cth;

  db_.pivot_length_ = 0.0;


  db_.global_door_closed_angle_ = db_.door_frame_global_yaw_;
  db_.global_door_open_angle_ = angles::normalize_angle(db_.door_frame_global_yaw_ + door.rot_dir*M_PI/2.0);

  db_.rot_dir_ = door.rot_dir;
  db_.init();

  //Set default desired door angle to door open position
  vector<int> desired_door_angles;
  desired_door_angles.resize(1);
  desired_door_angles[0] = angles::to_degrees(db_.global_door_open_angle_);
  printf("\n\nDoor hinge position: %f %f %f\n",hinge_global_x,hinge_global_y,hinge_global_z);
  printf("Door edge  position: %f %f %f\n",edge_global_x,edge_global_y,edge_global_z);
  printf("Door global yaw: %f\n",db_.door_frame_global_yaw_);
  printf("Rotation direction: %d\n",db_.rot_dir_);
  printf("Handle position in door frame: %f %f\n",db_.door_handle_position_.x,db_.door_handle_position_.y);
  printf("Global door open angle: %f\n",db_.global_door_open_angle_);
  SetDesiredDoorAngles(desired_door_angles);
}

void EnvironmentNAVXYTHETADOOR::setRobotProperties(const double &min_workspace_radius, 
                                                   const double &max_workspace_radius, 
                                                   const double &min_workspace_angle, 
                                                   const double &max_workspace_angle,
                                                   const robot_msgs::Point32 &robot_shoulder_position)
{
  db_.arm_min_workspace_radius_ = min_workspace_radius;
  db_.arm_max_workspace_radius_ = max_workspace_radius;

  db_.arm_max_workspace_angle_ = max_workspace_angle;
  db_.arm_min_workspace_angle_ = min_workspace_angle;

  db_.robot_shoulder_position_.x = robot_shoulder_position.x;
  db_.robot_shoulder_position_.y = robot_shoulder_position.y;


//	EnvNAVXYTHETALATCfg.FootprintPolygon = perimeterptsV;

  db_.footprint_.resize(EnvNAVXYTHETALATCfg.FootprintPolygon.size()); 

  for(int i=0; i < (int) EnvNAVXYTHETALATCfg.FootprintPolygon.size(); i++)
  {
    db_.footprint_[i].x = EnvNAVXYTHETALATCfg.FootprintPolygon[i].x;
    db_.footprint_[i].y = EnvNAVXYTHETALATCfg.FootprintPolygon[i].y;
  }

  printf("\n\nRobot properties\n");
  printf("Arm workspace angles: (min,max): %f %f\n",db_.arm_min_workspace_angle_,db_.arm_max_workspace_angle_);
  printf("Arm workspace radii: (min,max): %f %f\n",db_.arm_min_workspace_radius_,db_.arm_max_workspace_radius_);
  printf("Shoulder position: %f %f\n",db_.robot_shoulder_position_.x,db_.robot_shoulder_position_.y);
  printf("\n\n"); 
}

template<>
void EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAVLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

	if(stateID == EnvNAVXYTHETALAT.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d Theta=%d DoorInterval=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta, (int)HashEntry->door_intervalind);
    else
    	fprintf(fOut, "%.3f %.3f %.3f %d\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETALATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,EnvNAVXYTHETALATCfg.cellsize_m), 
		DiscTheta2Cont(HashEntry->Theta, NAVXYTHETALAT_THETADIRS), (int)HashEntry->door_intervalind);

}


//returns the stateid if success, and -1 otherwise
template<>
int EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::SetGoal(double x_m, double y_m, double theta_rad, unsigned char door_interval){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	printf("env: setting goal to %.3f %.3f %.3f %d (%d %d %d %d)\n", x_m, y_m, theta_rad, door_interval, x, y, theta, door_interval);

	if(!IsWithinMapCell(x,y))
	{
		printf("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
		printf("WARNING: goal configuration is invalid\n");
	}

    EnvNAVLATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, door_interval)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, door_interval);
    }

	//need to recompute start heuristics?
	if(EnvNAVXYTHETALAT.goalstateid != OutHashEntry->stateID)
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal

    EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

	EnvNAVXYTHETALATCfg.EndX_c = x;
	EnvNAVXYTHETALATCfg.EndY_c = y;
	EnvNAVXYTHETALATCfg.EndTheta = theta;
	desired_door_intervalindex = door_interval;

    return EnvNAVXYTHETALAT.goalstateid;    

}

//TODO-cont. here

//returns the stateid if success, and -1 otherwise
template<>
int EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::SetStart(double x_m, double y_m, double theta_rad, unsigned char door_interval){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m); 
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	if(!IsWithinMapCell(x,y))
	{
		printf("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
		return -1;
	}

	printf("env: setting start to %.3f %.3f %.3f %d (%d %d %d %d)\n", x_m, y_m, theta_rad, door_interval, x, y, theta, door_interval);

    if(!IsValidConfiguration(x,y,theta))
	{
		printf("WARNING: start configuration %d %d %d is invalid\n", x,y,theta);
	}

    EnvNAVLATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, door_interval)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, door_interval);
    }

	//need to recompute start heuristics?
	if(EnvNAVXYTHETALAT.startstateid != OutHashEntry->stateID)
		bNeedtoRecomputeStartHeuristics = true;

	//set start
    EnvNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
	EnvNAVXYTHETALATCfg.StartX_c = x;
	EnvNAVXYTHETALATCfg.StartY_c = y;
	EnvNAVXYTHETALATCfg.StartTheta = theta;
	desired_door_intervalindex = door_interval;


    return EnvNAVXYTHETALAT.startstateid;    

}


template<>
void EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::GetCoordFromState(int stateID, int& x, int& y, int& theta, unsigned char& door_interval) const {
  EnvNAVLATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  door_interval = HashEntry->door_intervalind;
}

template<>
int EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::GetStateFromCoord(int x, int y, int theta, unsigned char door_interval) {

   EnvNAVLATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta, door_interval)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, door_interval);
    }
    return OutHashEntry->stateID;
}


template<>
int EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath, 
																							vector<unsigned char>* door_intervalindPath)
{
	vector<EnvNAVXYTHETALATAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
	int targetx_c, targety_c, targettheta_c, targetdoorinterval;
	int sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval;

	//printf("checks=%ld\n", checks);

	xythetaPath->clear();
	door_intervalindPath->clear();


#if DEBUG
	fprintf(fDeb, "converting stateid path into coordinates:\n");
#endif

	for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
	{
		int sourceID = stateIDPath->at(pind);
		int targetID = stateIDPath->at(pind+1);

		//get successors and pick the target via the cheapest action
		SuccIDV.clear();
		CostV.clear();
		actionV.clear();
		GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

		//get source coordinates
		GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval);
		

		int bestcost = INFINITECOST;
		int bestsind = -1;

#if DEBUG
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targetdoorinterval);
		fprintf(fDeb, "looking for %d %d %d %d -> %d %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval,
					targetx_c, targety_c, targettheta_c, targetdoorinterval, SuccIDV.size()); 
#endif

		for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
		{

#if DEBUG
		int x_c, y_c, theta_c, doorint;
		GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, doorint);
		fprintf(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c, doorint); 
#endif

			if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
			{
				bestcost = CostV[sind];
				bestsind = sind;
			}
		}
		if(bestsind == -1)
		{
			printf("ERROR: successor not found for transition:\n");
			GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targetdoorinterval);
			printf("%d %d %d %d -> %d %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval,
					targetx_c, targety_c, targettheta_c, targetdoorinterval); 
			exit(1);
		}

		//now push in the actual path
		double sourcex, sourcey;
		sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
		for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++)
		{
			//translate appropriately
			EnvNAVXYTHETALAT3Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
			intermpt.x += sourcex;
			intermpt.y += sourcey;

#if DEBUG
			int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
			int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
			fprintf(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", 
				intermpt.x, intermpt.y, intermpt.theta, 
				nx, ny, 
				ContTheta2Disc(intermpt.theta, NAVXYTHETALAT_THETADIRS), EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
			if(ipind == 0) fprintf(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
			else fprintf(fDeb, "\n");
#endif

			//store
			xythetaPath->push_back(intermpt);
		}
	}
}



template<>
EnvNAVXYTHETADOORHashEntry_t*  EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::GetHashEntry(int X, int Y, int Theta, unsigned char door_interval)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Theta);
	
#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( Coord2StateIDHashTable[binid][ind]->X == X 
			&& Coord2StateIDHashTable[binid][ind]->Y == Y
			&& Coord2StateIDHashTable[binid][ind]->Theta == Theta
			&& Coord2StateIDHashTable[binid][ind]->door_intervalind == door_interval)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return Coord2StateIDHashTable[binid][ind];
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


template<>
EnvNAVXYTHETADOORHashEntry_t* EnvironmentNAVXYTHETADOOR<EnvNAVXYTHETADOORHashEntry_t>::CreateNewHashEntry(int X, int Y, int Theta, unsigned char door_interval) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVLATHashEntry_t* HashEntry = new EnvNAVLATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
	HashEntry->door_intervalind = door_interval;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta); 

	//insert the entry into the bin
	Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND];
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		printf("ERROR in Env... function: last state has incorrect stateID\n");
		exit(1);	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}



void EnvironmentNAVXYTHETADOOR::GetPathMinDoorAngle(const std::vector<EnvNAVXYTHETALAT3Dpt_t> &path, std::vector<double> &angle, std::vector<double> &angle_cost)
{
  angle.resize(path.size());
  angle_cost.resize(path.size());

  for(unsigned int i = 0; i < path.size(); i++) 
  {
    double door_angle;
    double door_angle_cost;
    if(this->GetMinCostDoorAngle(path.at(i).x, path.at(i).y, path.at(i).theta,door_angle,door_angle_cost))
    {
      angle.at(i) = door_angle;
      angle_cost.at(i) = door_angle_cost;
    }
    else
    {
      angle.at(i) = 0.0;
      angle_cost.at(i) = INFINITECOST;
    }
  }
}
