/*
 * box_tracker.h
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#ifndef EVAL_SAVER_H_
#define EVAL_SAVER_H_

#include "ros/ros.h"
#include "planar_objects/MocapEvalObservations.h"

namespace planar_objects
{

class EvalSaver
{
public:
  ros::NodeHandle nh;

  // MESSAGES - INCOMING
  ros::Subscriber eval_sub;
  MocapEvalObservationsConstPtr eval_msg;

  // Constructor
  EvalSaver();

  // Callbacks
  void evalCallback(const MocapEvalObservations::ConstPtr& observations);
};

}

int main(int argc, char** argv);

#endif /* BOX_TRACKER_H_ */
