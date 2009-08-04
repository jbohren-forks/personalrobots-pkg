/*
 * articulation_models.h
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#ifndef ARTICULATION_MODELS_H_
#define ARTICULATION_MODELS_H_

#include "track_utils.h"

namespace planar_objects {

class ManifoldModel {
public:
  btBoxTrack* track;
  ManifoldModel(btBoxTrack* track): track(track) {}
  virtual void findParameters() {}
  virtual double getRotationalError();
  virtual double getTranslationalError();
  virtual void getError(double &err_trans,double &err_rot);
  virtual size_t getDOFs() = 0;
  virtual std::vector< std::pair<double,double> > getRange();
  virtual std::vector<double> getConfiguration( btTransform transform) = 0;
  virtual btTransform getPrediction( std::vector<double> q ) = 0;
};

class PrismaticModel:public ManifoldModel {
public:
  btTransform center;
  btVector3 direction;
  double qMin, qMax;
  PrismaticModel(btBoxTrack* track);
  void findParameters();
  size_t getDOFs();
  std::vector<double> getConfiguration( btTransform transform);
  btTransform getPrediction( std::vector<double> configuration );
};

}

#endif /* ARTICULATION_MODELS_H_ */
