/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>

#include "amcl_odom.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLOdom::AMCLOdom(pf_matrix_t& drift) : AMCLSensor()
{
  this->action_pdf = NULL;
  this->time = 0.0;
  this->drift = drift;
}


#if 0
////////////////////////////////////////////////////////////////////////////////
// Get the current odometry reading
//AMCLSensorData *AMCLOdom::GetData(void)
// Process message for this interface
int AMCLOdom::ProcessMessage(QueuePointer &resp_queue,
                                     player_msghdr * hdr,
                                     void * idata)
{
  pf_vector_t pose;
  AMCLOdomData *ndata;

  if(!Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                            PLAYER_POSITION2D_DATA_STATE, this->odom_addr))
  {
    return -1;
  }

  player_position2d_data_t* data = reinterpret_cast<player_position2d_data_t*> (idata);

  // Compute new robot pose
  pose.v[0] = data->pos.px;
  pose.v[1] = data->pos.py;
  pose.v[2] = data->pos.pa;

  //printf("getdata %.3f %.3f %.3f\n",
  	 //pose.v[0], pose.v[1], pose.v[2]);

  ndata = new AMCLOdomData;

  ndata->sensor = this;
  ndata->time = hdr->timestamp;

  ndata->pose = pose;
  ndata->delta = pf_vector_zero();

  this->time = hdr->timestamp;

  AMCL.Push(ndata);

  return 0;
}
#endif


////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  AMCLOdomData *ndata;
  pf_vector_t x;
  pf_matrix_t cx;
  double ux, uy, ua;
  ndata = (AMCLOdomData*) data;

  /*
  printf("odom: %f %f %f : %f %f %f\n",
         ndata->pose.v[0], ndata->pose.v[1], ndata->pose.v[2],
         ndata->delta.v[0], ndata->delta.v[1], ndata->delta.v[2]);
  */

  // See how far the robot has moved
  x = ndata->delta;

  // Odometric drift model
  // This could probably be improved
  ux = this->drift.m[0][0] * x.v[0];
  uy = this->drift.m[1][1] * x.v[1];
  ua = this->drift.m[2][0] * fabs(x.v[0])
    + this->drift.m[2][1] * fabs(x.v[1])
    + this->drift.m[2][2] * fabs(x.v[2]);

  cx = pf_matrix_zero();
  cx.m[0][0] = ux * ux;
  cx.m[1][1] = uy * uy;
  cx.m[2][2] = ua * ua;

  //printf("x = %f %f %f\n", x.v[0], x.v[1], x.v[2]);

  // Create a pdf with suitable characterisitics
  this->action_pdf = pf_pdf_gaussian_alloc(x, cx);

  // Update the filter
  pf_update_action(pf, (pf_action_model_fn_t) ActionModel, this);

  // Delete the pdf
  pf_pdf_gaussian_free(this->action_pdf);
  this->action_pdf = NULL;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// The action model function (static method)
void
AMCLOdom::ActionModel(AMCLOdom *self, pf_sample_set_t* set)
{
  int i;
  pf_vector_t z;
  pf_sample_t *sample;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    z = pf_pdf_gaussian_sample(self->action_pdf);
    sample->pose = pf_vector_coord_add(z, sample->pose);
    sample->weight = 1.0 / set->sample_count;
  }
}


#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Setup the GUI
void AMCLOdom::SetupGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the GUI
void AMCLOdom::ShutdownGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}

#endif
