#include <ros/node.h>
#include <smartScan.h>
#include <fstream>
#include "grasp_module/object_detector_srv.h"
#include "grasp_module/grasp_planner_srv.h"

class Service : public ros::node
{
public:
	Service() : ros::node("gm_test_client"){}

	bool callObjDet(grasp_module::object_detector_srv::request &req,
			grasp_module::object_detector_srv::response &res) {
		return ros::service::call("object_detector_srv",req,res);
	}

	bool callGraspPlan(grasp_module::grasp_planner_srv::request &req,
			   grasp_module::grasp_planner_srv::response &res) {
		return ros::service::call("grasp_planner_srv",req,res);
	}
	
};

/*! This little executable testst the behavior of the
    grasp_module_node. Start the grasp_module_node, then fire this
    guy. As an argument, pass it a file that can be read in by a
    SmartScan::readFromFile(...)
 */
int main(int argc, char **argv)
{
	if (argc < 2) {
		//pass the name of a file from which a scan can be loaded
		fprintf(stderr,"Usage: gm_test filename\n");
		return 0;
	}

	SmartScan scan;

	// Load the test data 
	std::fstream fs;
	fs.open(argv[1],std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file: %s\n",argv[1]);
		return 0;
	}

	if (!scan.readFromFile(fs)) {
		fprintf(stderr,"Failed to read scan from file: %s\n",argv[1]);
		return 0;
	}
	fs.close();
	fprintf(stderr,"Test scan read from file. Size: %d points\n",scan.size());

	// Initialize ros
	ros::init(argc, argv);
	Service serv;


	// Set up query for object detection
	grasp_module::object_detector_srv::request req;
	grasp_module::object_detector_srv::response res;
	req.cloud = scan.getPointCloud();

	// Use all default parameters
	req.connectedThreshold = req.minComponentSize = -1;
	req.grazingThreshold = req.grazingRadius = req.grazingNbrs = -1;
	req.planeRemovalThreshold = req.planeFitThreshold = -1;

	// We want all objects defined by more than 100 points
	req.minComponentSize = 100;

	//send the query
	if (!serv.callObjDet(req,res) ) {
		fprintf(stderr,"Service call failed\n");
		ros::fini();
		return 0;
	}

	//display the objects
	fprintf(stderr,"%d object(s) found:\n",res.get_objects_size());
	std::vector<grasp_module::object_msg> objects;
	res.get_objects_vec(objects);
	for (int i=0; i<(int)objects.size(); i++) {
		fprintf(stderr,"  Centroid: %f %f %f\n",objects[i].centroid.x,
			objects[i].centroid.y, objects[i].centroid.z);
		fprintf(stderr,"  Axis 1: %f %f %f\n", objects[i].axis1.x,
			objects[i].axis1.y, objects[i].axis1.z);
		fprintf(stderr,"  Axis 2: %f %f %f\n", objects[i].axis2.x,
			objects[i].axis2.y, objects[i].axis2.z);
		fprintf(stderr,"  Axis 3: %f %f %f\n", objects[i].axis3.x,
			objects[i].axis3.y, objects[i].axis3.z);

		//and call the grasp planner
		grasp_module::grasp_planner_srv::request greq;
		grasp_module::grasp_planner_srv::response gres;

		greq.obj = objects[i];
		greq.resolution = 8; //set other resolution if desired

		//send the query
		if (!serv.callGraspPlan(greq,gres) ) {
			fprintf(stderr,"Service call failed\n");
			ros::fini();
			return 0;
		}
		fprintf(stderr,"  %d grasps found:\n",gres.get_graspPoints_size());
		std::vector<grasp_module::graspPoint_msg> grasps;
		gres.get_graspPoints_vec(grasps);
		for(int j=0; j<(int)grasps.size(); j++) {
			fprintf(stderr,"    %f %f %f\n",grasps[j].frame.xt, grasps[j].frame.yt,
				grasps[j].frame.zt);
			fprintf(stderr,"    %f %f %f %f\n",grasps[j].frame.w, grasps[j].frame.xr,
				grasps[j].frame.yr, grasps[j].frame.zr);
			fprintf(stderr,"    Quality: %f\n\n",grasps[j].quality);
		}
		fprintf(stderr,"\n");
	}
		
	ros::fini();
	return 0;
}
