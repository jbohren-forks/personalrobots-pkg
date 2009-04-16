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


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}


//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAVXYTHETADOORLAT::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (HashTableSize-1);
}

void EnvironmentNAVXYTHETADOORLAT::GetActionCost(int SourceX, int SourceY, int SourceTheta, 
						 int SourceDoorIntervalIndex,
						 EnvNAVXYTHETALATAction_t* action,
						 int* pCosttoDoorInterval0, int* pCosttoDoorInterval1)
{

    sbpl_2Dcell_t cell;
    vector<int> doorangleV, prevdoorangleV;
    vector<int> dooranglecostV, prevdooranglecostV;
    vector<unsigned char> doorangleintV, prevdoorangleintV;
    int i;

	
    if(!IsValidCell(SourceX, SourceY))
      {
	printf("\n\n\nReturning infinite cost\n\n\n");
	*pCosttoDoorInterval0 = INFINITECOST;
	*pCosttoDoorInterval1 = INFINITECOST;
	return;
      }
    //compute world frame offset for the action
    double sourcex = DISCXY2CONT(SourceX, EnvNAVXYTHETALATCfg.cellsize_m);
    double sourcey = DISCXY2CONT(SourceY, EnvNAVXYTHETALATCfg.cellsize_m);

    //iterate over intermediate poses and compute the cost multiplier due to door
    //active intervals indicate the interval that is currently reachable
    *pCosttoDoorInterval0 = 0;
    *pCosttoDoorInterval1 = 0;
    bool interval_active[2];
    interval_active[0] = interval_active[1] = false; //both inactive
    interval_active[SourceDoorIntervalIndex] = true; //active interval
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
	doorangleintV.clear();
//        printf("\n\n\n Getting action cost\n\n\n");
        GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV, &doorangleintV);
//        printf("Size angles: %d, costs: %d\n",doorangleV.size(),dooranglecostV.size());


        if(i > 0){
	  //go over the previous interval and pick the common value with the smallest cost
	  int mincost[2];
	  mincost[0] = mincost[1] = INFINITECOST;
	  for(int cind = 0; cind < (int)doorangleV.size(); cind++){
	    for(int pind = 0; pind < (int) prevdoorangleV.size(); pind++){
	      if(doorangleV[cind] == prevdoorangleV[cind] && 
		 interval_active[prevdoorangleintV[cind]] == true){
		//activate new interval if possible
		interval_active[doorangleintV[cind]] = true;
		//track the best cost
		if(__max(dooranglecostV[cind], prevdooranglecostV[pind]) < mincost[doorangleintV[cind]])
		  mincost[doorangleintV[cind]] = __max(dooranglecostV[cind], prevdooranglecostV[pind]);
	      }
	    }//prevdoorangles
	  }//current door angles
	  if(mincost[0] == INFINITECOST)
	    interval_active[0] = false; //nothing is reachable in this interval
	  if(mincost[1] == INFINITECOST)
	    interval_active[1] = false; //nothing is reachable in this interval

	  //is the worst case cost (a bit incorrect: should take min only if intervals overlap, otherwise it should be
	  //separate cost for each interval)
	  doorcostmultiplier = __max(__min(mincost[0], mincost[1]), doorcostmultiplier);
	    
	}
	else{
	  //NOTE: it would be more efficient to return intervalindex = 2 to indicate overlap. then it would be linear
	  for(int cind = 0; cind < (int)doorangleV.size(); cind++){
	    for(int c2ind = cind+1; c2ind < (int)doorangleV.size(); c2ind++){
	      if(doorangleV[cind] == doorangleV[c2ind] && 
		 (doorangleintV[cind] == SourceDoorIntervalIndex || doorangleintV[c2ind] == SourceDoorIntervalIndex)){
		interval_active[doorangleintV[cind]] = true;
		interval_active[doorangleintV[c2ind]] = true;
	      }	      
	    }
	  }
	}//else

	//    printf("\n\n\n Skipping previous step\n\n\n");
	if(doorcostmultiplier >= INFINITECOST){
	  *pCosttoDoorInterval0 = INFINITECOST;
	  *pCosttoDoorInterval1 = INFINITECOST;
	  return;
	}


        //store the old interval
        prevdoorangleV = doorangleV;
	prevdoorangleintV = doorangleintV;
        prevdooranglecostV = dooranglecostV;

    }   
 
  
    int currentmaxcost = 0;
    for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
      {
	//get the cell in the map
	cell = action->intersectingcellsV.at(i);
	cell.x = cell.x + SourceX;
	cell.y = cell.y + SourceY;
	
	//check validity
	if(!IsValidCell(cell.x, cell.y)){
	  *pCosttoDoorInterval0 = INFINITECOST;
	  *pCosttoDoorInterval1 = INFINITECOST;
	  return;
	}
	
	if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
	  currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
      }
    
    //to ensure consistency of h2D:
    currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
    if(!IsValidCell(SourceX + action->dX, SourceY + action->dY)){
      *pCosttoDoorInterval0 = INFINITECOST;
      *pCosttoDoorInterval1 = INFINITECOST;
      return;
    }
    
    currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);
    
    //use cell cost as multiplicative factor
    if(interval_active[0] == true)    
      *pCosttoDoorInterval0 = action->cost*(currentmaxcost+1)*(doorcostmultiplier + 1); 
    else
      *pCosttoDoorInterval0 = INFINITECOST;
    if(interval_active[1] == true)     
       *pCosttoDoorInterval1 =  action->cost*(currentmaxcost+1)*(doorcostmultiplier + 1); 
    else
      *pCosttoDoorInterval1 = INFINITECOST;

    if(*pCosttoDoorInterval0 == INFINITECOST && *pCosttoDoorInterval1 == INFINITECOST)
      {
	printf("ERROR: both costtodoorintervals are infinite\n");
	exit(1);
      }

}


void EnvironmentNAVXYTHETADOORLAT::GetValidDoorAngles(EnvNAVXYTHETALAT3Dpt_t worldrobotpose3D, 
						      vector<int>* doorangleV, 
						      vector<int>* dooranglecostV,
						      vector<unsigned char>* doorangleintV)
{

  //TODO for Sachin: set doorangleintV vector. It would actually be computationally faster if you return 2 
  //whenever an angle belongs to both intervals (as opposed to returning 2 different entries)
  //Let me know if you can do it, and then I'll change my code to support it

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
void EnvironmentNAVXYTHETADOORLAT::SetDesiredDoorAngles(vector<int> desired_door_anglesV)
{
  std::vector<int> desired_door_angles_local;
  desired_door_angles_local.resize(desired_door_anglesV.size());
  //set the vector
  db_.getDesiredDoorAngles(desired_door_anglesV, desired_door_angles_local);

  this->desired_door_anglesV.clear();
  this->desired_door_anglesV = desired_door_anglesV;

  printf("desired door angles are set to %d values\n", this->desired_door_anglesV.size());
}

//returns minimum cost angle within the specified door_angle_interval
int EnvironmentNAVXYTHETADOORLAT::MinCostDesiredDoorAngle(int x, int y, int theta, int door_intervalind)
{

  vector<int> doorangleV;
  vector<int> dooranglecostV;
  vector<unsigned char> doorangleintV;

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
  doorangleintV.clear();
  GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV, &doorangleintV);

  //go over the interval and pick the desired angle with the smallest cost
  int mincost = INFINITECOST;
  int bestangle = -1;
  for(int cind = 0; cind < (int)doorangleV.size(); cind++){
    for(int dind = 0; dind < (int) desired_door_anglesV.size(); dind++){
      if(doorangleintV[cind] == door_intervalind &&
	 doorangleV[cind] == desired_door_anglesV[dind] && dooranglecostV[cind] < mincost){
	mincost = dooranglecostV[cind];
	bestangle = desired_door_anglesV[dind];
      }
    }
  }

  return mincost;
}

//returns mininum cost angle within the specified door_angle_interval
bool EnvironmentNAVXYTHETADOORLAT::GetMinCostDoorAngle(double x, double y, double theta, 
						       unsigned char door_interval,
						       double &angle, double &door_angle_cost)
{
  vector<int> doorangleV;
  vector<int> dooranglecostV;
  vector<unsigned char> doorangleintV;

  //get the world 3d robot pose
  EnvNAVXYTHETALAT3Dpt_t point3D;
  point3D.x = x;
  point3D.y = y;
  point3D.theta = theta;

  //get the door intervals together with the costs
  doorangleV.clear();
  dooranglecostV.clear();
  doorangleintV.clear();
  GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV, &doorangleintV);

  //go over the interval and pick the angle with the smallest cost
  int mincost = INFINITECOST;
  int bestangle = -INFINITECOST;
  for(int cind = 0; cind < (int)doorangleV.size(); cind++)
  {
//    printf("angles[%d]: %d %d\n",cind,doorangleV[cind],dooranglecostV[cind]);
      if(doorangleintV[cind] == door_interval && dooranglecostV[cind] < mincost)
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
void EnvironmentNAVXYTHETADOORLAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, 
								       vector<int>* CostV, 
								       vector<EnvNAVXYTHETALATAction_t*>* actionV /*=NULL*/)
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
  EnvNAVXYTHETADOORHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
    
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
    //int newdoorintervals = 0; //0 - same, 1 - new interval, 2 - both old and new
    int costtodoorinterval[2];
    GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->door_intervalind, nav3daction, 
		  &costtodoorinterval[0], &costtodoorinterval[1]);
	
    //check that the out state is not goal in term of desired door_angles
    //iterate over target door intervals
    for(int dind = 0; dind < 2; dind++){

      //check reachability
      if(costtodoorinterval[dind] >= INFINITECOST)
	continue; //impossible to reach

      int mincostofdesireddoorangle = MinCostDesiredDoorAngle(newX,newY,newTheta,dind);
      if (mincostofdesireddoorangle < INFINITECOST)
	{
	  //insert the goal (NOTE: we ignore the final door cost)
	  SuccIDV->push_back(EnvNAVXYTHETALAT.goalstateid);
	  CostV->push_back(costtodoorinterval[dind]);
	  if(actionV != NULL)
	    actionV->push_back(nav3daction);	  
	}
      else{		
	EnvNAVXYTHETADOORHashEntry_t* OutHashEntry;
	
	if((OutHashEntry = GetHashEntry(newX, newY, newTheta, dind)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = CreateNewHashEntry(newX, newY, newTheta, dind);
	  }
	SuccIDV->push_back(OutHashEntry->stateID);
	CostV->push_back(costtodoorinterval[dind]);
	if(actionV != NULL)
	  actionV->push_back(nav3daction);
      }
    }//over door intervals
  }
    
#if TIME_DEBUG
  time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAVXYTHETADOORLAT::setDoorDiscretizationAngle(const double &door_angle_discretization_interval)
{
  db_.door_angle_discretization_interval_ = door_angle_discretization_interval;
}


void EnvironmentNAVXYTHETADOORLAT::setDoorProperties(const robot_msgs::Door &door, 
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


void EnvironmentNAVXYTHETADOORLAT::setRobotProperties(const double &min_workspace_radius, 
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


void EnvironmentNAVXYTHETADOORLAT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
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

	EnvNAVXYTHETADOORHashEntry_t* HashEntry = StateID2CoordTable[stateID];

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
int EnvironmentNAVXYTHETADOORLAT::SetGoal(double x_m, double y_m, double theta_rad, 
					  unsigned char door_interval /*=ENVNAVXYTHETADOOR_DEFAULTDESIREDDOORINTERVALIND*/){

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
  
  EnvNAVXYTHETADOORHashEntry_t* OutHashEntry;
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


//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETADOORLAT::SetStart(double x_m, double y_m, double theta_rad, 
					   unsigned char door_interval /*=ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND*/){

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

  EnvNAVXYTHETADOORHashEntry_t* OutHashEntry;
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
  start_door_intervalindex = door_interval;


  return EnvNAVXYTHETALAT.startstateid;    

}


void EnvironmentNAVXYTHETADOORLAT::GetCoordFromState(int stateID, int& x, int& y, int& theta, 
										unsigned char& door_interval) const {
  EnvNAVXYTHETADOORHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  door_interval = HashEntry->door_intervalind;
}

void EnvironmentNAVXYTHETADOORLAT::GetCoordFromState(int stateID, int& x, int& y, int& theta) const {
  EnvNAVXYTHETADOORHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
}


int EnvironmentNAVXYTHETADOORLAT::GetStateFromCoord(int x, int y, int theta, 
						    unsigned char door_interval/*=ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND*/) {

   EnvNAVXYTHETADOORHashEntry_t* OutHashEntry;
   if((OutHashEntry = GetHashEntry(x, y, theta, door_interval)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta, door_interval);
    }
    return OutHashEntry->stateID;
}


void EnvironmentNAVXYTHETADOORLAT::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, 
								   vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath,								      vector<unsigned char>* door_intervalindPath)
{

	vector<EnvNAVXYTHETALATAction_t*> actionV;
	vector<int> CostV;
	vector<int> SuccIDV;
	int targetx_c, targety_c, targettheta_c;
	int sourcex_c, sourcey_c, sourcetheta_c;
	unsigned char sourcedoorinterval, targetdoorinterval;

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
		GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targetdoorinterval);
		
		int bestcost = INFINITECOST;
		int bestsind = -1;

#if DEBUG
		fprintf(fDeb, "looking for %d %d %d %d -> %d %d %d %d (numofsuccs=%d)\n", 
			sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval,
			targetx_c, targety_c, targettheta_c, targetdoorinterval, SuccIDV.size()); 
#endif

		for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
		{

#if DEBUG
		  int x_c, y_c, theta_c, doorint;
		  GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, doorint);
		  fprintf(fDeb, "succ: %d %d %d %d\n", x_c, y_c, theta_c, doorint); 
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
			printf("%d %d %d %d -> %d %d %d %d\n", 
			       sourcex_c, sourcey_c, sourcetheta_c, sourcedoorinterval,
			       targetx_c, targety_c, targettheta_c, targetdoorinterval); 
			exit(1);
		}

		//now push in the actual path
		double sourcex, sourcey;
		sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
		int currentintervalind = sourcedoorinterval;
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
		  
		  //get the door angles and if there is one with the interval of the goal configuration return it
		  //NOTE: relies on not having switch back and forth in between intervals within one action 
		  //- should probably fix this
		  if(currentintervalind != targetdoorinterval){
		    vector<int> doorangleV, dooranglecostV;
		    vector<unsigned char> doorangleintV;		    
		    GetValidDoorAngles(intermpt, &doorangleV, &dooranglecostV, &doorangleintV);
		    for(int d1ind = 0; d1ind < (int)doorangleV.size(); d1ind++){
		      for(int d2ind = d1ind+1; d2ind < (int)doorangleV.size(); d2ind++){
			if(doorangleV[d1ind] == doorangleV[d2ind] && doorangleintV[d1ind] != doorangleintV[d2ind])
			  currentintervalind = targetdoorinterval; //transition into target interval			 
			  }
		    }
		  }
		  door_intervalindPath->push_back(currentintervalind);		  
		}
	}//over pind
}



EnvNAVXYTHETADOORHashEntry_t* EnvironmentNAVXYTHETADOORLAT::GetHashEntry(
								       int X, int Y, int Theta, 
								       unsigned char door_interval)
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
	  if(Coord2StateIDHashTable[binid][ind]->X == X 
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


EnvNAVXYTHETADOORHashEntry_t*EnvironmentNAVXYTHETADOORLAT::CreateNewHashEntry(
											  int X, int Y, int Theta, 
											  unsigned char door_interval) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETADOORHashEntry_t* HashEntry = new EnvNAVXYTHETADOORHashEntry_t;

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



void EnvironmentNAVXYTHETADOORLAT::GetPathMinDoorAngle(const std::vector<EnvNAVXYTHETALAT3Dpt_t> &path, 
						       const std::vector<unsigned char> &doorangleintpath,
									 std::vector<double> &angle, std::vector<double> &angle_cost)
{
  angle.resize(path.size());
  angle_cost.resize(path.size());

  for(unsigned int i = 0; i < path.size(); i++) 
  {
    double door_angle;
    double door_angle_cost;
    if(this->GetMinCostDoorAngle(path.at(i).x, path.at(i).y, path.at(i).theta,doorangleintpath.at(i),
				 door_angle,door_angle_cost))
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


int EnvironmentNAVXYTHETADOORLAT::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvNAVXYTHETADOORHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	EnvNAVXYTHETADOORHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	

	return (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAVXYTHETALATCfg.nominalvel_mpersecs);	

}


int EnvironmentNAVXYTHETADOORLAT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAVXYTHETALAT.goalstateid);

}


int EnvironmentNAVXYTHETADOORLAT::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif


	if(bNeedtoRecomputeStartHeuristics)
	{
		grid2Dsearch->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeStartHeuristics = false;
		printf("2dsolcost_infullunits=%d\n", (int)(grid2Dsearch->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c)
			/EnvNAVXYTHETALATCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	EnvNAVXYTHETADOORHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearch->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X, HashEntry->Y));
		

#if DEBUG
	fprintf(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs); 

}

int EnvironmentNAVXYTHETADOORLAT::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}

void EnvironmentNAVXYTHETADOORLAT::InitializeEnvironment()
{
	EnvNAVXYTHETADOORHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	HashTableSize = 64*1024; //should be power of two
	Coord2StateIDHashTable = new vector<EnvNAVXYTHETADOORHashEntry_t*>[HashTableSize];
	
	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta, 
				       start_door_intervalindex);
	EnvNAVXYTHETALAT.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta,
				       desired_door_intervalindex);
	EnvNAVXYTHETALAT.goalstateid = HashEntry->stateID;

	//initialized
	EnvNAVXYTHETALAT.bInitialized = true;

}
