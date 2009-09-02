/*
 * articulation_learner.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "articulation_learner.h"

#include "vis_utils.h"
#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

#define sqr(a) ((a)*(a))
#define DELTA 0.000001

int main(int argc, char **argv) {
	ros::init(argc, argv, "articulation_learner");

	planar_objects::ArticulationLearner node;
	ros::spin();

	return 0;
}

namespace planar_objects {

// Constructor
ArticulationLearner::ArticulationLearner() :
	sync(&ArticulationLearner::syncCallback, this) {
	nh.param("~visualize", visualize, true);
	nh.param("~verbose", verbose, true);

	nh.param("~dist_vis", dist_vis, 0.02);

	nh.param("~thres_trans", thres_trans, 0.1);
	nh.param("~thres_rot", thres_rot, 360.0 / 180.0 * M_PI);

	// subscribe to topics
	tracks_sub = nh.subscribe("box_tracker/tracks", 1, sync.synchronize(
			&ArticulationLearner::tracksCallback, this));

	oldLines = 0;
	newLines = 0;

	// advertise topics
	visualization_pub = nh.advertise<visualization_msgs::Marker> (
			"~visualization_marker", 0);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud> ("~planes", 1);
	articulated_pub = nh.advertise<planar_objects::ArticulatedObjects> (
			"~articulated_objects", 1);
}

void ArticulationLearner::tracksCallback(const BoxTracks::ConstPtr& tracks_msg) {
	this->tracks_msg = tracks_msg;
}

void ArticulationLearner::syncCallback() {
	ROS_INFO("ArticulationLearner::syncCallback(), received %d tracks",tracks_msg->get_tracks_size());
	setVisualization(&visualization_pub, &cloud_pub, tracks_msg->header);

	tracks.clear(); // forget everything
	for (size_t i = 0; i < tracks_msg->tracks.size(); i++) {
		for (int j = 0; j < 1; j++) {
			tracks.push_back(btBoxTrack(tracks_msg->tracks[i],
					tracks_msg->header.stamp, j));
		}
	}

	createModels();
	updateModels();
	thresholdModels();
	selectSimpleModels();
	//  suppressUnarticulatedModels();
	selectModels();

	if (visualize) {
		//    visualizeTracks();
		visualizeModels();
		removeOldLines();
	}

	publishArticulatedObjects();

	releaseModels();
}

void ArticulationLearner::releaseModels() {
	for (size_t i = 0; i < models.size(); i++) {
		delete (models[i]);
	}
	models.clear();
}

void ArticulationLearner::createModels() {
	//  if(tracks.size()<1) return;
	//  size_t best_i = 0;
	//  for(size_t i=1;i<tracks.size();i++) {
	//	  if(tracks[i].obs_history.size() > tracks[best_i].obs_history.size())
	//		  best_i = i;
	//  }
	//  models.push_back(new RotationalModel(&tracks[best_i]));

	// create models for all tracks
	for (size_t i = 0; i < tracks.size(); i++) {
		if (tracks[i].obs_history.size() == 0)
			continue;
		models.push_back(new RigidModel(&tracks[i]));
		models.push_back(new PrismaticModel(&tracks[i]));
		models.push_back(new RotationalModel(&tracks[i]));
	}
}

void ArticulationLearner::updateModels() {
	for (size_t i = 0; i < models.size(); i++) {
		models[i]->findParameters(&visualization_pub);
		models[i]->computeError();
	}
}

void ArticulationLearner::thresholdModels() {
	size_t i = 0;
	while (i < models.size()) {
		double err_rot = DBL_MAX, err_trans = DBL_MAX;
		models[i]->getError(err_trans, err_rot);
		cout << "model " << i << " error is: " << err_rot << " " << err_trans
				<< " from " << models[i]->track->obs_history.size() << "obs."
				<< endl;
		if (err_trans > thres_trans || err_rot > thres_rot) {
			models.erase(models.begin() + i);
		} else {
			i++;
		}
	}
	cout << "valid models: " << models.size() << endl;
}

void ArticulationLearner::suppressUnarticulatedModels() {
	size_t i = 0;
	while (i < models.size()) {
		if (!models[i]->isArticulated(thres_trans * 3, thres_rot * 3)) {
			models.erase(models.begin() + i);
		} else {
			i++;
		}
	}
	cout << "articulated models: " << models.size() << endl;
}

void ArticulationLearner::selectSimpleModels() {
	std::map<int, size_t> m;
	for (size_t i = 0; i < models.size(); i++) {
		size_t complexity = models[i]->getComplexityClass();
		if ((m.find(models[i]->track->id) == m.end())
				|| (m[models[i]->track->id] > complexity))
			m[models[i]->track->id] = complexity;
	}

	size_t i = 0;
	while (i < models.size()) {
		size_t complexity = models[i]->getComplexityClass();
		if (m[models[i]->track->id] != complexity) {
			cout << "deleting model " << i << endl;
			models.erase(models.begin() + i);
		} else {
			i++;
		}
	}

	cout << "selected simple models: " << models.size() << endl;
}

void ArticulationLearner::selectModels() {
	std::map<int, double> m;
	for (size_t i = 0; i < models.size(); i++) {
		double logp = models[i]->getLoglikelihood(thres_trans, thres_rot);
		cout << "model " << i << " error is: " << models[i]->err_rot << " "
				<< models[i]->err_trans << " log p=" << logp << " from "
				<< models[i]->track->obs_history.size() << "obs." << endl;
		if ((m.find(models[i]->track->id) == m.end())
				|| (m[models[i]->track->id] < logp))
			m[models[i]->track->id] = logp;
	}

	size_t i = 0;
	while (i < models.size()) {
		double logp = models[i]->getLoglikelihood(thres_trans, thres_rot);
		cout << "model " << i << " id=" << models[i]->track->id
				<< " error is: " << models[i]->err_rot << " "
				<< models[i]->err_trans << " highest logp="
				<< m[models[i]->track->id] << " log p=" << logp << " from "
				<< models[i]->track->obs_history.size() << "obs." << endl;
		if (m[models[i]->track->id] > logp) {
			cout << "deleting model " << i << endl;
			models.erase(models.begin() + i);
		} else {
			i++;
		}
	}

	cout << "selected models: " << models.size() << endl;
}

void ArticulationLearner::visualizeTracks() {
	//  cout << "observations_msg->header.stamp="<<(long)observations_msg->header.stamp.toSec() << endl;
	for (size_t j = 0; j < tracks.size(); j++) {
		//    cout << "tracks[j].obs_history.rbegin()->stamp="<<(long)tracks[j].obs_history.rbegin()->stamp.toSec() << endl;
		//    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + ros::Duration(1.00) ) continue;
		for (size_t i = 0; i < tracks[j].obs_history.size(); i++) {
			visualizeLines(tracks[j].obs_history[i].visualize(), newLines++,
					HSV_to_RGB(j / (double) tracks.size(), MIN(1.0, 0.2*tracks[j].obs_history.size() ), 1.0
					//((i+3)/((double)tracks[j].obs_history.size()+3))
					//                          1.0
					));
		}
	}
}

void ArticulationLearner::visualizeModels() {
	for (size_t i = 0; i < models.size(); i++) {
		cout << "model " << i << ", track " << models[i]->track->id
				<< ", type " << typeid(*models[i]).name() << endl;
		std::vector<std::pair<double, double> > range = models[i]->getRange();
		std::vector<double> q_mean;
		q_mean.resize(models[i]->getDOFs());
		for (size_t d = 0; d < models[i]->getDOFs(); d++)
			q_mean[d] = (range[d].first + range[d].second) / 2;
		for (size_t d = 0; d < models[i]->getDOFs(); d++) {
			std::vector<double> q = q_mean;
			int n = (int) ((range[0].second - range[0].first) / dist_vis);
			//      cout << "n="<<n<<endl;
			for (size_t j = 0; j < (size_t) n; j++) {
				//        q[d] = ((n-j-1)*range[d].first + j*range[d].second)/(n-1.0);
				q[d] = ((n - j - 1) * range[d].first + j * range[d].second)
						/ (n + 1);
				btBoxObservation predObs =
						models[i]->getPredictedObservation(q);
				//        cout <<
				//          "w="<<predObs.w <<
				//          " h="<<predObs.h <<
				//          " x="<<predObs.tf.getOrigin().x()<<
				//          " y="<<predObs.tf.getOrigin().y()<<
				//          " z="<<predObs.tf.getOrigin().z()<<
				//          " rx="<<predObs.tf.getRotation().x()<<
				//          " ry="<<predObs.tf.getRotation().y()<<
				//          " rz="<<predObs.tf.getRotation().z()<<
				//          " rw="<<predObs.tf.getRotation().w()<<
				//          endl;
				//        cout <<
				//        " x="<<models[i]->center.getOrigin().x()<<
				//        " y="<<models[i]->center.getOrigin().y()<<
				//        " z="<<models[i]->center.getOrigin().z()<<
				//        " rx="<<models[i]->center.getRotation().x()<<
				//        " ry="<<models[i]->center.getRotation().y()<<
				//        " rz="<<models[i]->center.getRotation().z()<<
				//        " rw="<<models[i]->center.getRotation().w()<<
				//        " dx="<<models[i]->direction.x()<<
				//        " dy="<<models[i]->direction.y()<<
				//        " dz="<<models[i]->direction.z()<<
				//          endl;
				visualizeLines(predObs.visualize(), newLines++, HSV_to_RGB(j
						/ ((double) n), 1.0, 1.0));
			}
		}

		if (models[i]->track->obs_history.size() > 0) {
			std::vector<double> q = models[i]->getConfiguration(
					models[i]->track->obs_history.rbegin()->tf);
			btBoxObservation predObs = models[i]->getPredictedObservation(q);
			double r = 1.0;
			if (q.size() > 0)
				r = (q[0] - range[0].first)
						/ (range[0].second - range[0].first);
			visualizeLines(predObs.visualize(), newLines++, HSV_to_RGB(r, 1.0,
					1.0), 0.01);
		}
	}
}

void ArticulationLearner::removeOldLines() {
	LineVector lines;
	for (int l = newLines; l < oldLines; l++) {
		visualizeLines(lines, l, 0, 0, 0);
	}
	oldLines = newLines;
	newLines = 0;
}

void ArticulationLearner::publishArticulatedObjects() {
	ArticulatedObjects msg;
	msg.header = tracks_msg->header;

	msg.set_obj_size(models.size());

	for (size_t i = 0; i < models.size(); i++) {
		msg.obj[i].model_class = typeid( *models[i] ).name();
		msg.obj[i].track_id = models[i]->track->id;
		msg.obj[i].n_obs = models[i]->track->obs_history.size();
		msg.obj[i].error_translational = models[i]->getTranslationalError();
		msg.obj[i].error_rotational = models[i]->getRotationalError();
		msg.obj[i].loglikelihood = models[i]->getLoglikelihood(thres_trans,
				thres_rot);
		msg.obj[i].dof = models[i]->getDOFs();
		msg.obj[i].complexity = models[i]->getComplexityClass();

		if (models[i]->track->obs_history.size() > 0) {
			msg.obj[i].set_q_current_size(models[i]->getDOFs());
			msg.obj[i].set_Jx_current_size(models[i]->getDOFs());
			msg.obj[i].set_Jy_current_size(models[i]->getDOFs());
			msg.obj[i].set_Jz_current_size(models[i]->getDOFs());

			std::vector<double> q = models[i]->getConfiguration(
					models[i]->track->obs_history.rbegin()->tf);

			btBoxObservation predObs = models[i]->getPredictedObservation(q);
			for (size_t j = 0; j < models[i]->getDOFs(); j++) {
				msg.obj[i].q_current[j] = q[j];

				std::vector<double> qDelta = q;
				qDelta[j] += DELTA;
				btBoxObservation predDelta =
						models[i]->getPredictedObservation(qDelta);
				btTransform jacobianDelta = predObs.tf.inverseTimes(
						predDelta.tf);

				msg.obj[i].Jx_current[j] = jacobianDelta.getOrigin().x()
						/ DELTA;
				msg.obj[i].Jy_current[j] = jacobianDelta.getOrigin().y()
						/ DELTA;
				msg.obj[i].Jz_current[j] = jacobianDelta.getOrigin().z()
						/ DELTA;
			}

		}

		articulated_pub.publish(msg);
	}

}

}
