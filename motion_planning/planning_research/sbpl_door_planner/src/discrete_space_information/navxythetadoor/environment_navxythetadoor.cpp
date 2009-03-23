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

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	
	if(!IsValidCell(SourceX, SourceY))
		return INFINITECOST;

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
        GetValidDoorAngles(point3D, &doorangleV, &dooranglecostV);

        if(i > 0){
            //go over the previous interval and pick the common value with the smallest cost
            int mincost = INFINITECOST;
            int bestprevangle = -1;
            for(int cind = 0; cind < (int)doorangleV.size(); cind++){
                for(int pind = 0; pind < (int) doorangleV.size(); pind++){
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

    doorangleV->push_back(1);
    doorangleV->push_back(2);
    dooranglecostV->push_back(0);
    dooranglecostV->push_back(3);

}

