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

//! \author Vijay Pradeep

/****
 * This driver provides a ros-interface for the Phase Space Impulse positioning system.
 * www.PhaseSpace.com
 */

#include "ros/node.h"


// PhaseSpace API
#include "owl.h"

// Messages
#include "phase_space/PhaseSpaceSnapshot.h"
#include "phase_space/PhaseSpaceMarker.h"


#define negativeCheck(result, msg) \
  if (result < 0) \
  { \
    printf("PhaseSpaceNode:: %s (result=%i)\n", msg, result) ; \
    assert(result < 0) ; \
  }



static const int NUM_MARKERS = 64 ;


namespace phase_space
{

class PhaseSpaceNode : public ros::node
{
public:
  
  PhaseSpaceNode() : ros::node("phase_space")
  {
    advertise<PhaseSpaceSnapshot>("phase_space_snapshot", 48) ;
    
  }
  
  void startOwlClient()
  {
    int result ;
    
    result = owlInit("10.0.0.24", 0) ;                                          // Flag term is 0, since we currently have no modifiers for startup
    negativeCheck(result, "Error initializing owl connection, owlInit()") ;
    
  }
  
  void addDefaultMarkers()
  {
    owlTrackeri(0, OWL_CREATE, OWL_POINT_TRACKER) ;

    if(!owlGetStatus())
    {
      owl_print_error("A) error in point tracker setup", owlGetError()) ;
      return ;
    }
  
    // Simply add the first NUM_MARKERS led-IDs to list of markers to track
    for (int i=0; i<NUM_MARKERS; i++)
    {
      owlMarkeri(MARKER(0, i), OWL_SET_LED, i) ;
    }
    
    if(!owlGetStatus())
    {
      owl_print_error("B) error in point tracker setup", owlGetError()) ;
      return ;
    }

    owlTracker(0, OWL_ENABLE);

    // flush requests and check for errors
    if(!owlGetStatus())
    {
      owl_print_error("C) error in point tracker setup", owlGetError()) ;
      return ;
    }
  }
  
  void startStreaming()
  {
    // set to the max frequency
    owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);

    // start streaming
    owlSetInteger(OWL_STREAMING, OWL_ENABLE);
    
    // Turn on timestamps
    owlSetInteger(OWL_TIMESTAMP, OWL_ENABLE);
  }
  
  bool spin()
  {
    OWLMarker markers[NUM_MARKERS] ;
    
    while (ok())             // While the node has not been shutdown
    {
      usleep(1) ;
      int n = owlGetMarkers(markers, NUM_MARKERS) ;

      int timeVal[3] ;
      timeVal[0] = 0 ;
      timeVal[1] = 0 ;
      timeVal[2] = 0 ;
      int ntime = owlGetIntegerv(OWL_TIMESTAMP, timeVal) ;
      
      if (ntime < 0)
        printf("ERROR: ntime=%i\n", ntime) ;

      int err ;
      if((err = owlGetError()) != OWL_NO_ERROR)
      {
        owl_print_error("error", err);
        break;
      }

      if(n > 0)
      {
        int marker_count = 0 ;
        
        PhaseSpaceSnapshot snapshot ;
        snapshot.set_markers_size(NUM_MARKERS) ;
        snapshot.frameNum = markers[0].frame ;
        
        for(int i=0; i < n; i++)
        {
          if(markers[i].cond > 0)
          {
            copyMarkerToMessage(markers[i], snapshot.markers[marker_count]) ;
            marker_count++ ;
          }
        }
        
        snapshot.set_markers_size(marker_count) ;
        
        //printf("%d marker(s):\n", n);
        //for(int i = 0; i < n; i++)
        // {
        //  if(markers[i].cond > 0)
        //    printf("%d) FRAME=%i (%f %f %f)\n", i, markers[i].frame, markers[i].x, markers[i].y, markers[i].z) ;
        //}
        //printf("\n") ;
        
        
        publish("phase_space_snapshot", snapshot) ;
        
      }
    }

    return true;
  }
  
  void shutdownOwlClient()
  {
    owlDone() ;             // OWL API-call to perform system cleanup before client termination
  }
  
private:
  
  void copyMarkerToMessage(const OWLMarker& owl_marker, PhaseSpaceMarker& msg_marker)
  {
    msg_marker.id         = owl_marker.id ;

    // Don't forget that owl markers are in millimeters, whereas all ROS data types are in meters!
    msg_marker.location.x = owl_marker.x / 1000.0 ;
    msg_marker.location.y = owl_marker.y / 1000.0 ;
    msg_marker.location.z = owl_marker.z / 1000.0 ;
    msg_marker.condition  = owl_marker.cond ;
  }
  
  void owl_print_error(const char *s, int n)
  {
    if(n < 0) printf("%s: %d\n", s, n);
    else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
    else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
    else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
    else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
    else printf("%s: 0x%x\n", s, n);
  }
} ;

}

using namespace phase_space ;

int main(int argc, char** argv)
{
  ros::init(argc, argv);

  PhaseSpaceNode phase_space_node ;

  phase_space_node.startOwlClient() ;
  
  phase_space_node.addDefaultMarkers() ;
  
  phase_space_node.startStreaming() ;
  
  printf("Phase Space Client Running...\n") ;
  printf("(Press Crtl-C to quit)\n") ;
  
  phase_space_node.spin() ;

  phase_space_node.shutdownOwlClient() ;

  ros::fini() ;
  
 

//  int result ;
//  result = owlInit("10.0.0.24", OWL_SLAVE | OWL_POSTPROCESS) ;
  
  return 0 ;
}
