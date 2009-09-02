/*
 * box_detector.cpp
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <boost/thread.hpp>
#include "track_utils.h"

#include "planar_objects/BoxObservations.h"
#include "planar_objects/BoxTracks.h"

using namespace std;

namespace planar_objects {

template<class MessageType> class PeriodicPublisher {
public:
	PeriodicPublisher(std::string topic, double frequency) :
		topic_(topic), frequency_(frequency), publish_thread_(NULL) {
		publish_thread_ = new boost::thread(boost::bind(
				&PeriodicPublisher::publishLoop, this));
	}

	~PeriodicPublisher() {
		if (publish_thread_ != NULL) {
			publish_thread_->join();
			delete publish_thread_;
		}
	}

	void setMessage(MessageType msg) {
		ROS_DEBUG("Updating message");
		msg_ = msg;
	}

	void publishLoop() {
		ros::NodeHandle n;
		ros::Rate r(frequency_);
		ros::Publisher pub = n.advertise<MessageType> (topic_, 1);
		while (n.ok()) {
			pub.publish(msg_);
			r.sleep();
		}
	}

	std::string topic_;
	double frequency_;
	MessageType msg_;
	boost::thread* publish_thread_;
};

class BoxPublisher {
public:
	std::vector<planar_objects::BoxObservation> obs_seq;

	BoxPublisher(PeriodicPublisher<planar_objects::BoxTracks>& pp,string filename) :
		pp_(pp), count_(0) {
		ros::NodeHandle n;
		timer_ = n.createTimer(ros::Duration(1/pp.frequency_), boost::bind(
				&BoxPublisher::updateMessage, this));

		std::ifstream infile (filename.c_str());
		while(!infile.eof()) {
			string l;
			getline(infile,l);
			stringstream ls(l);

			planar_objects::BoxObservation obs;
			//  obs.header = cloud_->header;
			ls >> obs.transform.translation.x;
			ls >> obs.transform.translation.y;
			ls >> obs.transform.translation.z;
			if(!ls.eof()) {
				ls >> obs.transform.rotation.x;
				ls >> obs.transform.rotation.y;
				ls >> obs.transform.rotation.z;
				ls >> obs.transform.rotation.w;
			} else {
				obs.transform.rotation.x = 0;
				obs.transform.rotation.y = 0;
				obs.transform.rotation.z = 1;
				obs.transform.rotation.w = 0;
			}
			if(!ls.eof()) {
				ls >> obs.w;
				ls >> obs.h;
			} else {
				obs.w = 0.01;
				obs.h = 0.01;
			}

			cout << "obs["<<obs_seq.size()<<"] "<<endl;
			cout << " t.x="<<obs.transform.translation.x;
			cout << " t.y="<<obs.transform.translation.y;
			cout << " t.z="<<obs.transform.translation.z<<endl;
			cout << " r.x="<<obs.transform.rotation.x;
			cout << " r.y="<<obs.transform.rotation.y;
			cout << " r.z="<<obs.transform.rotation.z;
			cout << " r.w="<<obs.transform.rotation.w<<endl;
			cout << " width="<<obs.w;
			cout << " height="<<obs.h<<endl;

			obs.precision = 1.00;
			obs.recall = 1.00;
			obs.plane_id = 0;
			obs_seq.push_back(obs);
		}

		ROS_INFO("read logfile %s, number of observations: %d",filename.c_str(),obs_seq.size());
	}

	void updateMessage() {
		BoxTracks tracks;
		tracks.set_tracks_size(1);
		tracks.tracks[0].id = 0;
//		tracks.tracks[0].set_obs_size(obs_seq.size()/2);
		tracks.tracks[0].set_obs_size(count_ % obs_seq.size());
		for(size_t i=0;i<tracks.tracks[0].get_obs_size();i++) {
			tracks.tracks[0].obs[i]=obs_seq[i];
		}
		ROS_INFO("Sending %d observations [0..%d] as a single track..", tracks.tracks[0].obs.size(),count_ % obs_seq.size());

		pp_.setMessage( tracks );
		count_++;
	}

private:
	PeriodicPublisher<planar_objects::BoxTracks>& pp_;
	int count_;
	ros::Timer timer_;
};

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "box_tracker");

	planar_objects::PeriodicPublisher<planar_objects::BoxTracks> pp(
			"~tracks", 5);

	planar_objects::BoxPublisher tester(pp,"advait-1.log");

	ros::spin();
	return (0);
}
