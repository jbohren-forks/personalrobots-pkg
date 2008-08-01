/*
 * sbpl_2dnav
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

//for timing
#include <sys/time.h>

//for map
#include <gdk-pixbuf/gdk-pixbuf.h>

// roscpp
#include <ros/node.h>

#include <std_srvs/StaticMap.h>

//sbpl headers file
#include <headers.h>

class Sbpl2DNav : public ros::node {
public:
  Sbpl2DNav(void) : ros::node("sbpl_2dnav") {
    std_srvs::StaticMap::request  req;
    std_srvs::StaticMap::response resp;
    puts("Requesting the map...");
    while(!ros::service::call("static_map", req, resp))
      {
	puts("request failed; trying again...");
	usleep(1000000);
      }
    printf("Received a %d X %d map @ %.3f m/pix\n",
	   resp.map.width,
	   resp.map.height,
	   resp.map.resolution);
    
    char* mapdata;
    int sx, sy;
    sx = resp.map.width;
    sy = resp.map.height;
    // Convert to player format
    mapdata = new char[sx*sy];
    for(int i=0;i<sx*sy;i++) {
      if(resp.map.data[i] == 0)
	mapdata[i] = 0;
      else if(resp.map.data[i] == 100)
	mapdata[i] = 1;
      else
	mapdata[i] = 0;
    }

    char* expdata;
    expdata = new char[sx*sy];
    memcpy(expdata,mapdata,sx*sy);
    int g = 3;
    //stupid 4-connected obstacle growth
    for(int i = g; i < sx-g; i++) {
      for(int j = g; j < sy-g; j++) {
	if(mapdata[i+j*sx] == 1) {
	  for(int k = i-g; k <= i+g; k++) {
	    for(int l = j-g; l <= j+g; l++) {
	      expdata[k+l*sx] = 1;
	    }
	  }
	}
      }
    }

    environment_nav2D.SetConfiguration(sx,
				       sy,
				       expdata,
				       0,0,
				       539,586);
    delete mapdata;
    environment_nav2D.InitGeneral();
    
    //Initialize MDP Info
    if(!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
      printf("ERROR: InitializeMDPCfg failed\n");
      exit(1);
    }
  }

  void DrawEnvMapWithPath(vector<int> path) {
    GdkPixbuf* pixbuf;
    GError* error = NULL;
    guchar* pixels;
    int p;
    int paddr;
    int i, j;

    // Initialize glib
    g_type_init();

    const EnvNAV2DConfig_t* env = environment_nav2D.GetEnvNavConfig();

    pixels = (guchar*)malloc(sizeof(guchar)*env->EnvWidth_c*env->EnvHeight_c*3);


    //this draws the obstacles
    p=0;
    for(j=env->EnvHeight_c-1;j>=0;j--) {
      for(i=0; i<env->EnvWidth_c; i++,p++) {
	paddr = p * 3;
	
	/*
	  if(i > 190 && i < 210 && j > 290 && j < 310) {
	  pixels[paddr] = 0;
	  pixels[paddr+1] = 0;
	  pixels[paddr+2] = 255;
	} else 
	*/
	if(env->Grid2D[i][j] == 1) {
	  pixels[paddr] = 0;
	  pixels[paddr+1] = 0;
	  pixels[paddr+2] = 0;
	} else {
	  pixels[paddr] = 255;
	  pixels[paddr+1] = 255;
	  pixels[paddr+2] = 255;
	}
      }
    }
    
    //now we draw the path
    for(unsigned int i = 0; i < path.size(); i++) {
      int px, py;
      environment_nav2D.GetCoordFromState(path[i], px, py);
      //std::cout << px << " " << py << std::endl;
      paddr = 3*(px+env->EnvWidth_c*(env->EnvHeight_c-py-1));
      pixels[paddr] = 0;
      pixels[paddr+1] = 255;
      pixels[paddr+2] = 0;
    }

    pixbuf = gdk_pixbuf_new_from_data(pixels, 
				      GDK_COLORSPACE_RGB,
				      0,8,
				      env->EnvWidth_c,
				      env->EnvHeight_c,
				      env->EnvWidth_c * 3,
				      NULL, NULL);
    
    gdk_pixbuf_save(pixbuf,"path.png","png",&error,NULL);
    gdk_pixbuf_unref(pixbuf);
    free(pixels);
  }

public:

  MDPConfig MDPCfg;
  EnvironmentNAV2D environment_nav2D;
    

};

int main(int argc, char **argv) {

  struct timeval timebefore;
  struct timeval timeafter;

  ros::init(argc, argv);
  
  Sbpl2DNav snav;
  
  sleep(1);
  
  double allocated_time_secs = 60.0*15; //in seconds

  vector<int> solution_stateIDs_V;
  ARAPlanner ara_planner(&snav.environment_nav2D, &snav.MDPCfg);
  gettimeofday(&timebefore,NULL);
  int bRet = ara_planner.replan(allocated_time_secs, &solution_stateIDs_V);
  gettimeofday(&timeafter,NULL);
  int best_seconds = timeafter.tv_sec-timebefore.tv_sec;
  int best_microseconds = timeafter.tv_usec-timebefore.tv_usec;
  double best_tt = best_seconds*1.0+(.000001)*(best_microseconds*1.0);
  
  std::cout << "Planning took " << best_tt << " seconds\n";

  if(bRet) {
    //print the solution
    printf("Solution is found\n");
  } else {
    printf("Solution does not exist\n");
  }

  snav.DrawEnvMapWithPath(solution_stateIDs_V);
    

  snav.shutdown();
}
