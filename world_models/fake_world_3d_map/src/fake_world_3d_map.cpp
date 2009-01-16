/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author Ioan Sucan */

/**

@mainpage

@htmlinclude ../manifest.html

@b fake_world_3d_map is a node that issues certain world maps as if
they were produced by world_3d_map. This is useful in testing.

<hr>
 
@section usage Usage
@verbatim
$ fake_world_3d_map map_number [standard ROS args]
@endverbatim


@par Example

@verbatim
$ fake_world_3d_map 1
@endverbatim

@par Notes

@b map_number is the index of the map to be published.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name/type):
- @b "world_3d_map"/PointCloud : point cloud describing the 3D environment

<hr>

Uses (name/type):
- None

Provides (name/type):
- None

**/

#include <ros/node.h> 
#include <ros/time.h> 
#include <std_msgs/PointCloud.h>
#include <random_utils/random_utils.h>
#include <cstdio>

class FakeWorld3DMap : public ros::Node
{
public:
    
    FakeWorld3DMap(void) : ros::Node("world_3d_map")
    {
	advertise<std_msgs::PointCloud>("world_3d_map", 1);
	random_utils::init(&m_rng);
    }
    
    ~FakeWorld3DMap(void)
    {
    }
    
    void sendMap(int index)
    {
	switch (index)
	{
	case 1:
	    oneObstacle();
	    break;
	case 2:
	    verticalObstacle();
	    break;
	case 3:
	    planeX();
	    break;
	default:
	    break;
	}
	m_toPublish.header.frame_id = "FRAMEID_MAP";
	m_toPublish.header.stamp = ros::Time::now();
	publish("world_3d_map", m_toPublish);
    }
    
    
private:
    
    void oneObstacle(void)
    {
	m_toPublish.set_pts_size(1);
	m_toPublish.pts[0].x = 0.6;
	m_toPublish.pts[0].y = 0.35;
	m_toPublish.pts[0].z = 0.75;
    }
    
    void verticalObstacle(void)
    {
	const int N = 30;
	const int M = 3;
	const int L = 3;
	
	m_toPublish.set_pts_size(N * M * L);

	for (int k = 0 ; k < L ; ++k)
	{
	    double x = 0.30 + 0.02 * k;
	    
	    for (int j = 0 ; j < M ; ++j)
	    {
		double y = -0.30 - 0.02 * j;
		
		for (int i = 0 ; i < N ; ++i)
		{
		    double z = 0.1 + i * 0.05;
		    m_toPublish.pts[i + j*N + k*N*M].x = x;
		    m_toPublish.pts[i + j*N + k*N*M].y = y;
		    m_toPublish.pts[i + j*N + k*N*M].z = z;
		}
	    }
	}	
    }
    
    void planeX(void)
    {
	//	for (int i = 0 ; i < 20 ; ++i)
	//	    for (int j = 0 ; j < 20 ;
    }
    
    std_msgs::PointCloud m_toPublish; 
    random_utils::rngState      m_rng;

};

void usage(const char *progname)
{
    printf("\nUsage: %s map_number [standard ROS args]\n", progname);
    printf("       \"map_number\" is the index of the map to be published.\n");
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    if (argc == 2)
    {
	FakeWorld3DMap *map = new FakeWorld3DMap();
	int index = 0;
	sscanf(argv[1], "%d", &index);	
	sleep(1);	
	map->sendMap(index);
	map->shutdown();
	delete map;
    }
    else
	usage(argv[0]);
    
    return 0;    
}
