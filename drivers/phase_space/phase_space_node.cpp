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

#include "phase_space_node.h"

using namespace phase_space ;

static const int MAX_NUM_MARKERS = 64 ;
static const int MAX_NUM_BODIES = 64 ;

static const bool DEBUG_ON = true ;

#define negativeCheck(result, msg) \
  if (result < 0) \
  { \
    printf("PhaseSpaceNode:: %s (result=%i)\n", msg, result) ; \
    assert(result < 0) ; \
  }

PhaseSpaceNode::PhaseSpaceNode() : ros::Node("phase_space")
{
  advertise<PhaseSpaceSnapshot>("phase_space_snapshot", 48) ;
}

PhaseSpaceNode::~PhaseSpaceNode()
{
  unadvertise("phase_space_snapshot") ;
}

void PhaseSpaceNode::startOwlClient()
{
  int result ;
  result = owlInit("10.0.0.24", OWL_SLAVE) ;
  negativeCheck(result, "Error initializing owl connection, owlInit()") ;
}

void PhaseSpaceNode::startStreaming()
{
  // start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE) ;

  // Enable Timestamps
  owlSetInteger(OWL_TIMESTAMP, OWL_ENABLE) ;
}

bool PhaseSpaceNode::spin()
{ 

  unsigned int prev_frame_num = -1 ;
  while (ok())             // While the node has not been shutdown
  {
    usleep(1) ;
    PhaseSpaceSnapshot snapshot ;

    snapshot.header.frame_id = "phase_space" ;

    int n_markers, n_bodies ;
    n_markers = grabMarkers(snapshot) ;
    n_bodies =  grabBodies(snapshot) ;
    
    if (prev_frame_num != snapshot.frameNum)            // Make sure that we're not reading stale data
    {
      prev_frame_num = snapshot.frameNum ;
      if (n_markers > 0 || n_bodies > 0)
      {
        grabTime(snapshot) ;
        publish("phase_space_snapshot", snapshot) ;
        
        if (snapshot.frameNum % 48 == 0)
          dispSnapshot(snapshot) ;
      }
    }
  }  
  return true ;
}

int PhaseSpaceNode::grabMarkers(PhaseSpaceSnapshot& snapshot)
{
  OWLMarker markers[MAX_NUM_MARKERS] ;

  int n = owlGetMarkers(markers, MAX_NUM_MARKERS) ;

  int err ;
  if((err = owlGetError()) != OWL_NO_ERROR)
  {
    owlPrintError("error", err) ;
    return 0 ;
  }

  if(n > 0)
  {
    int marker_count = 0 ;

    snapshot.set_markers_size(MAX_NUM_MARKERS) ;        // Make vector as large a possibl, and shrink once we know how many elements we actually have
    snapshot.frameNum = markers[0].frame ;              // //! \todo Need to do better frame num management

    for(int i=0; i < n; i++)
    {
      if(markers[i].cond > 0)                           // Check that server did not mess up in computing pose
      {
        copyMarkerToMessage(markers[i], snapshot.markers[marker_count]) ;
        marker_count++ ;
      }
    }

    snapshot.set_markers_size(marker_count) ;
    
  }
  return n ;
}

int PhaseSpaceNode::grabBodies(PhaseSpaceSnapshot& snapshot)
{
  OWLRigid bodies[MAX_NUM_BODIES] ;
  int n = owlGetRigids(bodies, MAX_NUM_BODIES) ;
  
  if (n > 0)
  {
    int body_count = 0 ;
    
    snapshot.set_bodies_size(MAX_NUM_MARKERS) ;
    snapshot.frameNum = bodies[0].frame ;               //! \todo Need to do better frame num management
    
    for (int i=0; i < n; i++)
    {
      if(bodies[i].cond > 0)                            // Check that server did not mess up in computing pose
      {
        copyBodyToMessage(bodies[i], snapshot.bodies[body_count]) ;
        body_count++ ;
      }
    }
    snapshot.set_bodies_size(body_count) ;
  }
  
  return n ;
}

void PhaseSpaceNode::grabTime(PhaseSpaceSnapshot& snapshot)
{
  int timeVal[3] ;
  timeVal[0] = 0 ;
  timeVal[1] = 0 ;
  timeVal[2] = 0 ;
  int ntime = owlGetIntegerv(OWL_TIMESTAMP, timeVal) ;

  if (ntime < 0)
    printf("ERROR: ntime=%i\n", ntime) ;
  
  snapshot.header.stamp = ros::Time((unsigned int) timeVal[1], (unsigned int) timeVal[2]*1000 ) ;
}


void PhaseSpaceNode::shutdownOwlClient()
{
  owlDone() ;             // OWL API-call to perform system cleanup before client termination
}
  
void PhaseSpaceNode::copyMarkerToMessage(const OWLMarker& owl_marker, PhaseSpaceMarker& msg_marker)
{
  msg_marker.id         = owl_marker.id ;

  // Don't forget that owl markers are in millimeters, whereas all ROS data types are in meters!
  msg_marker.location.x = owl_marker.x / 1000.0 ;
  msg_marker.location.y = owl_marker.y / 1000.0 ;
  msg_marker.location.z = owl_marker.z / 1000.0 ;
  msg_marker.condition  = owl_marker.cond ;
}

void PhaseSpaceNode::copyBodyToMessage(const OWLRigid& owl_body, PhaseSpaceBody& msg_body)
{
  msg_body.id = owl_body.id ;
  
  msg_body.pose.translation.x = owl_body.pose[0] / 1000.0 ;
  msg_body.pose.translation.y = owl_body.pose[1] / 1000.0 ;
  msg_body.pose.translation.z = owl_body.pose[2] / 1000.0 ;
  
  msg_body.pose.rotation.w    = owl_body.pose[3] ;
  msg_body.pose.rotation.x    = owl_body.pose[4] ;
  msg_body.pose.rotation.y    = owl_body.pose[5] ;
  msg_body.pose.rotation.z    = owl_body.pose[6] ;
  
  msg_body.condition = owl_body.cond ;
}
  
void PhaseSpaceNode::owlPrintError(const char *s, int n)
{
  if(n < 0)
    printf("%s: %d\n", s, n);
  else if(n == OWL_NO_ERROR)
    printf("%s: No Error\n", s);
  else if(n == OWL_INVALID_VALUE)
    printf("%s: Invalid Value\n", s);
  else if(n == OWL_INVALID_ENUM)
    printf("%s: Invalid Enum\n", s);
  else if(n == OWL_INVALID_OPERATION)
    printf("%s: Invalid Operation\n", s);
  else
    printf("%s: 0x%x\n", s, n);
}

void PhaseSpaceNode::dispSnapshot(const PhaseSpaceSnapshot& s)
{
  printf("rosTF_frame: %s   Frame #%u\n", s.header.frame_id.c_str(), s.frameNum) ;
  printf("  Time: %lf\n", s.header.stamp.to_double()) ;
  printf("  Markers: %u\n", s.get_markers_size()) ;
  
  unsigned int i = 0 ;
  for (i=0; i < s.get_markers_size(); i++)
  {
    printf("    %02u) (% 10.4f, % 10.4f, % 10.4f)\n", s.markers[i].id, s.markers[i].location.x, s.markers[i].location.y, s.markers[i].location.z) ;
  }

  for (; i<10; i++)
    printf("\n") ;
  
  
  printf("  Bodies: %u\n", s.get_bodies_size()) ;
  for (i=0; i < s.get_bodies_size(); i++)
  {
    printf("    %02u) Loc (% 10.4f, % 10.4f, % 10.4f)\n", s.bodies[i].id,
                                                          s.bodies[i].pose.translation.x,
                                                          s.bodies[i].pose.translation.y,
                                                          s.bodies[i].pose.translation.z) ;
    printf("        Rot (x=% 1.4f, y=% 1.4f, z=% 1.4f, w=% 1.4f)\n", s.bodies[i].pose.rotation.x,
                                                                     s.bodies[i].pose.rotation.y,
                                                                     s.bodies[i].pose.rotation.z,
                                                                     s.bodies[i].pose.rotation.w) ;
  }
  
  for (; i<2; i++)
    printf("\n\n") ;
  
  printf("\n") ;
}

int main(int argc, char** argv)
{

  printf("Initializing Ros...") ;
  ros::init(argc, argv);
  printf("Done\n") ;

  printf("Constructing phase_space_node...") ;
  PhaseSpaceNode phase_space_node ;
  printf("Done\n") ;

  printf("Trying to connect to OWL server...\n") ;  
  phase_space_node.startOwlClient() ;
  printf("Done\n") ;
  
  printf("Starting streaming...\n") ;
  phase_space_node.startStreaming() ;
  printf("Done\n") ;
  
  printf("Streaming started\n") ;
  
  printf("Phase Space Client Running...\n") ;
  printf("(Press Crtl-C to quit)\n") ;
  
  phase_space_node.spin() ;

  printf("Shutting Down OWL Client\n") ;
  phase_space_node.shutdownOwlClient() ;

  ros::fini() ;
  
  printf("**** Exiting ****\n") ;
  
  return 0 ;
}
