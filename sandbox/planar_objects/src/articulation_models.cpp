/*
 * articulation_models.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#include "articulation_models.h"
#include "Eigen/Core"
#include <Eigen/SVD>
#include <Eigen/Eigen>
#include "opencv/cv.h"

using namespace Eigen;

USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

namespace planar_objects
{

void ManifoldModel::findParameters()
{
  w = 0;
  h = 0;
  if (track->obs_history.size() < 2)
    return;

  double sum_w = 0, sum_h = 0;
  for (size_t j = 0; j < track->obs_history.size(); j++)
  {
    sum_w += track->obs_history[j].w;
    sum_h += track->obs_history[j].h;
  }
  w = sum_w / track->obs_history.size();
  h = sum_h / track->obs_history.size();
}

double ManifoldModel::getTranslationalError()
{
  double err_trans, err_rot;
  getError(err_trans, err_rot);
  return (err_trans);
}

double ManifoldModel::getRotationalError()
{
  double err_trans, err_rot;
  getError(err_trans, err_rot);
  return (err_rot);
}

bool ManifoldModel::isValid() {
  return(true);
}

void ManifoldModel::getError(double &err_trans, double &err_rot)
{
  err_rot = DBL_MAX;
  err_trans = DBL_MAX;
  if (track->obs_history.size() < 2 || !isValid())
    return;

  double sum_trans = 0, sum_rot = 0;
  for (size_t j = 0; j < track->obs_history.size(); j++)
  {
    btTransform pred = getPrediction(getConfiguration(track->obs_history[j].tf));
    btTransform error =
        track->obs_history[j].tf.inverseTimes(pred);
    sum_trans += error.getOrigin().length();
    sum_rot += error.getRotation().getAngle();
  }
  err_trans = sum_trans / track->obs_history.size();
  err_rot = sum_rot / track->obs_history.size();

  if(!isnormal(err_trans)) err_trans= DBL_MAX;
  if(!isnormal(err_rot)) err_rot= DBL_MAX;
}

std::vector<std::pair<double, double> > ManifoldModel::getRange()
{
  std::vector<std::pair<double, double> > range;
  range.resize(getDOFs());
  if (track->obs_history.size() == 0)
    return range;

  std::vector<double> q = getConfiguration(track->obs_history[0].tf);
  for (size_t i = 0; i < getDOFs(); i++)
  {
    range[i].first = q[i];
    range[i].second = q[i];
  }
  for (size_t j = 1; j < track->obs_history.size(); j++)
  {
    std::vector<double> q = getConfiguration(track->obs_history[j].tf);
    for (size_t i = 0; i < getDOFs(); i++)
    {
      range[i].first = MIN(range[i].first, q[i]);
      range[i].second = MAX(range[i].second, q[i]);
    }
  }

  return (range);
}

btBoxObservation ManifoldModel::getPredictedObservation(std::vector<double> q)
{
  btBoxObservation result;
  result.w = w;
  result.h = h;
  result.precision = 1.0;
  result.recall = 1.0;
  result.tf = getPrediction(q);
  return (result);
}

PrismaticModel::PrismaticModel(btBoxTrack* track) :
  ManifoldModel(track)
{
}

void PrismaticModel::findParameters()
{
  ManifoldModel::findParameters();

  if (track->obs_history.size() == 0)
  {
    ROS_ERROR("PrismaticModel::findParameters on empty history!!");
    return;
  }
  // find center
  btQuaternion o = track->obs_history[0].tf.getRotation();
  btVector3 t = track->obs_history[0].tf.getOrigin();

  for (size_t j = 1; j < track->obs_history.size(); j++)
  {
    o = o.slerp(track->obs_history[j].tf.getRotation(), 1 / (j + 1.0));
    t = t.lerp(track->obs_history[j].tf.getOrigin(), 1 / (j + 1.0));
  }

  center = btTransform(o, t);

  if (track->obs_history.size() < 6)
  {
    return;
  }

  // find line direction
  MatrixXf X_trans(track->obs_history.size(), 3);
  for (size_t j = 0; j < track->obs_history.size(); j++)
  {
    X_trans(j, 0) = center.inverseTimes(track->obs_history[j].tf).getOrigin().x();
    X_trans(j, 1) = center.inverseTimes(track->obs_history[j].tf).getOrigin().y();
    X_trans(j, 2) = center.inverseTimes(track->obs_history[j].tf).getOrigin().z();
  }
  //  std::cout << "Matrix X is:" << std::endl << std::endl << X_trans << std::endl << std::endl;

  VectorXf b(track->obs_history.size());
  VectorXf x(3);
  SVD<MatrixXf> svdOfA(X_trans);
  svdOfA.solve(b, &x);

  const Eigen::MatrixXf U = svdOfA.matrixU();
  const Eigen::MatrixXf V = svdOfA.matrixV();
  const Eigen::VectorXf S = svdOfA.singularValues();

  DiagonalMatrix<Eigen::VectorXf> S2(S);
  //
  //  std::cout << "Matrix U is:" << std::endl << std::endl << U << std::endl << std::endl;
  //  std::cout << "Matrix V is:" << std::endl << std::endl << V << std::endl << std::endl;
  //  std::cout << "Vector S is:" << std::endl << std::endl << S << std::endl << std::endl;
  //  std::cout << "Matrix S2 is:" << std::endl << std::endl << S2 << std::endl << std::endl;
  //  std::cout << "Vector x is:" << std::endl << std::endl << x << std::endl << std::endl;
  //  std::cout << "Vector b is:" << std::endl << std::endl << b << std::endl << std::endl;

  Eigen::MatrixXf Y_trans = V * S2;
  //  std::cout << "Matrix Y_trans is:" << std::endl << std::endl << Y_trans << std::endl << std::endl;

  direction = btVector3(V(0, 0), V(1, 0), V(2, 0)).normalize();

  //  cout <<"estimating q's:"<<endl;
  //  for(size_t j=0;j<track->obs_history.size();j++) {
  //    cout << "j="<<j<<" q[0]="<<getConfiguration(track->obs_history[j].tf)[0]<<endl;
  //  }
  //
  //  cout <<"translational error:"<<getTranslationalError()<<endl;
  //  cout <<"rotational error:"<<getRotationalError()<<endl;

  cout << "current range: " << getRange()[0].first << " .. " << getRange()[0].second << endl;
}

size_t PrismaticModel::getDOFs()
{
  return (1);
}

std::vector<double> PrismaticModel::getConfiguration(btTransform transform)
{
  std::vector<double> q;
  q.resize(1);
  q[0] = direction.dot(center.inverseTimes(transform).getOrigin());
  return (q);
}

btTransform PrismaticModel::getPrediction(std::vector<double> q)
{
  if (q.size() != 1)
    ROS_ERROR("PrismaticModel::getPrediction must have 1 DOF");
  return (center * btTransform(btQuaternion::getIdentity(), direction * q[0]));
}

bool PrismaticModel::isValid() {
  if(!isnormal(center.getOrigin().x())) return(false);
  if(!isnormal(center.getOrigin().y())) return(false);
  if(!isnormal(center.getOrigin().z())) return(false);
  if(!isnormal(center.getRotation().x())) return(false);
  if(!isnormal(center.getRotation().y())) return(false);
  if(!isnormal(center.getRotation().z())) return(false);
  if(!isnormal(center.getRotation().w())) return(false);
  if(!isnormal(direction.x())) return(false);
  if(!isnormal(direction.y())) return(false);
  if(!isnormal(direction.z())) return(false);
  return(true);
}

}
