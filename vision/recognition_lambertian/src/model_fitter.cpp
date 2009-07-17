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

// Author: Marius Muja

#include <boost/filesystem.hpp>

#include <ply.h>

#include <ros/ros.h>
#include "recognition_lambertian/ModelFit.h"

#include "robot_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "mapping_msgs/Object.h"

#include "tf/transform_datatypes.h"
#ifdef PUBLISH_GRASH_POSE
#include <tf/transform_broadcaster.h>
#endif

namespace bfs = boost::filesystem;
using namespace std;
using namespace robot_msgs;

namespace model_fit {


typedef struct Vertex {
  float x,y,z;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;

const char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


class PLYMesh
{
	/*** the PLY object ***/

	PlyFile *in_ply;
	int nverts,nfaces;
	Vertex vertex;
	Face face;
	PlyOtherProp *vert_other,*face_other;

public:

	inline void endian_swap(void* p)
	{
		unsigned int* x = (unsigned int*)p;

	    *x = (*x>>24) |
	        ((*x<<8) & 0x00FF0000) |
	        ((*x>>8) & 0x0000FF00) |
	        (*x<<24);
	}


	void readFromFile(const string& filename, mapping_msgs::Object& mesh)
	{
		ROS_INFO("Loading mesh file: %s", filename.c_str());
		int i,j;
		int elem_count;
		char *elem_name;

		/*** Read in the original PLY object ***/

		FILE* fin = fopen(filename.c_str(), "rb");

		if (fin==NULL)  {
			ROS_ERROR("Cannot read file: %s", filename.c_str());
			return;
		}

		in_ply = read_ply (fin);

		float version;
		int file_type;
		get_info_ply(in_ply, &version, &file_type);

		/* examine each element type that is in the file (vertex, face) */

		for (i = 0; i < in_ply->num_elem_types; i++) {

			/* prepare to read the i'th list of elements */
			elem_name = setup_element_read_ply (in_ply, i, &elem_count);

			if (equal_strings ((char*)"vertex", elem_name)) {
				nverts = elem_count;
				mesh.vertices.resize(nverts);

				/* set up for getting vertex elements */
				/* (we want x,y,z) */

				setup_property_ply (in_ply, &vert_props[0]);
				setup_property_ply (in_ply, &vert_props[1]);
				setup_property_ply (in_ply, &vert_props[2]);

				/* we also want normal information if it is there (nx,ny,nz) */

				//		      for (j = 0; j < in_ply->elems[i]->nprops; j++) {
				//			PlyProperty *prop;
				//			prop = in_ply->elems[i]->props[j];
				//			if (equal_strings ("nx", prop->name)) {
				//			  setup_property_ply (in_ply, &vert_props[3]);
				//			  has_nx = 1;
				//			}
				//			if (equal_strings ("ny", prop->name)) {
				//			  setup_property_ply (in_ply, &vert_props[4]);
				//			  has_ny = 1;
				//			}
				//			if (equal_strings ("nz", prop->name)) {
				//			  setup_property_ply (in_ply, &vert_props[5]);
				//			  has_nz = 1;
				//			}
				//		      }

				/* also grab anything else that we don't need to know about */

				vert_other = get_other_properties_ply (in_ply,
						offsetof(Vertex,other_props));

				/* grab the vertex elements*/
				for (j = 0; j < elem_count; j++) {
					get_element_ply (in_ply, (void *) &vertex);
					if (file_type==PLY_BINARY_BE) {
						endian_swap(&vertex.x);
						endian_swap(&vertex.y);
						endian_swap(&vertex.z);
					}

					mesh.vertices[j].x = vertex.x / 1000;
					mesh.vertices[j].y = vertex.y / 1000;
					mesh.vertices[j].z = vertex.z / 1000;
				}
			}
			else if (equal_strings ((char*)"face", elem_name)) {

				nfaces = elem_count;
				mesh.triangles.resize(nfaces*3);

				/* set up for getting face elements */
				/* (all we need are vertex indices) */

				setup_property_ply (in_ply, &face_props[0]);
				face_other = get_other_properties_ply (in_ply,
						offsetof(Face,other_props));

				/* grab all the face elements and place them in our list */

				for (j = 0; j < elem_count; j++) {
					get_element_ply (in_ply, (void *) &face);
					if (file_type==PLY_BINARY_BE) {
						for (int k=0;k<face.nverts;++k) {
							endian_swap(&face.verts[k]);
						}
					}


					if (face.nverts!=3) {
						ROS_WARN("Mesh contains non triangle faces");
					}
					else {
						mesh.triangles[3*j] = face.verts[0];
						mesh.triangles[3*j+1] = face.verts[1];
						mesh.triangles[3*j+2] = face.verts[2];
					}
				}
			}
			else  /* all non-vertex and non-face elements are grabbed here */
				get_other_element_ply (in_ply);
		}

		close_ply (in_ply);

		ROS_INFO("File contains %d vertices and %d faces", nverts, nfaces);
	}


};




class ModelFitSet;

class TemplateModel {
	int x_res, y_res, z_res;
	float x_min, y_min, z_min;
	float x_max, y_max, z_max;
	float x_d, y_d, z_d;
	float truncate_value;

	float *grid;
	mapping_msgs::Object mesh_;

	string name_;

public:
	TemplateModel()
	{

	}

	~TemplateModel()
	{
		delete[] grid;
	}

	void load(const string& file, const string& name);

	string name()
	{
		return name_;
	}

	Pose graspPose()
	{
		// TODO: fix this, models annotated with pose
		Pose pose;
		pose.position.x = 0;
		pose.position.y = 0;
		pose.position.z = 0.05;


		btMatrix3x3 rotation;
		rotation.setIdentity();
		rotation.setEulerZYX(M_PI/2,0,0);
		btQuaternion orientation;
		rotation.getRotation(orientation);

		pose.orientation.x = orientation.x();
		pose.orientation.y = orientation.y();
		pose.orientation.z = orientation.z();
		pose.orientation.w = orientation.w();

		return pose;
	}

	void getExtents(Point32& low_extent, Point32& high_extent)
	{
		Pose pose = graspPose();

		low_extent.x = pose.position.x-x_min;
		low_extent.y = pose.position.y-y_min;
		low_extent.z = pose.position.z-z_min;

		high_extent.x = x_max-pose.position.x;
		high_extent.y = y_max-pose.position.y;
		high_extent.z = z_max-pose.position.z;

	}

	mapping_msgs::Object objectMesh()
	{
		return mesh_;
	}


	void show(const ros::Publisher& publisher, const Point32& location, float fit_score);

	bool in_bounds(int x, int y, int z)
	{
		return (x>=0 && x<x_res && y>=0 && y<y_res && z>=0 && z<z_res);
	}

	void fitPointCloud(const PointCloud& cloud, const Point32& location, ModelFitSet& mfs);

	void findBestFit(const PointCloud& cloud, ModelFitSet& mfs);
};


class ModelFitSet
{
public:
	struct ModelFit
	{
		TemplateModel* model_;
		Point32 location_;
		float score_;
		float max_dist_;

		ModelFit() : model_(NULL)
		{
		}

		ModelFit(TemplateModel* model, const Point32& location, float score, float max_dist) :
			model_(model), location_(location), score_(score), max_dist_(max_dist)
		{
		}

		float score() {
			return score_;
		}

		Pose graspPose()
		{
			Pose object_pose = objectPose();
			Pose model_grasp_pose = model_->graspPose();
			tf::Pose tf_object_pose;
			tf::Pose tf_model_grasp_pose;

			tf::poseMsgToTF(object_pose, tf_object_pose);
			tf::poseMsgToTF(model_grasp_pose, tf_model_grasp_pose);

			tf::Pose tf_grasp_pose = tf_object_pose*tf_model_grasp_pose;
			Pose grasp_pose;

			tf::poseTFToMsg(tf_grasp_pose, grasp_pose);


			return grasp_pose;
		}

		Pose objectPose()
		{
			Pose pose;
			pose.position.x = location_.x;
			pose.position.y = location_.y;
			pose.position.z = location_.z;

			pose.orientation.x = 0;
			pose.orientation.y = 0;
			pose.orientation.z = 0;
			pose.orientation.w = 1.0;

			return pose;
		}

	};
	int size_;
	int count_;
	ModelFit* best_fit_;

	ModelFitSet(int size) : size_(size), count_(0)
	{
		best_fit_ = new ModelFit[size_];
	}

	void add(TemplateModel* model, const Point32& location, float score, float max_dist)
	{
		ModelFit crt(model,location, score, max_dist);

		if (count_ < size_) {
			best_fit_[count_++] = crt;
		}
		else if (best_fit_[size_-1].score()>crt.score()) {
			int i=size_-1;
			while (i>0 && best_fit_[i].score()>crt.score()) {
				best_fit_[i] = best_fit_[i-1];
				i--;
			}
			best_fit_[i] = crt;
		}
	}


};



void TemplateModel::load(const string& file, const string& name)
{
	name_ = name;

	FILE* f;
	f = fopen(file.c_str(),"r");
	if (f==NULL) {
		ROS_ERROR("Cannot open template file: %s", file.c_str());
	}
	fscanf(f,"%d %d %d ", &x_res, &y_res, &z_res );
	fscanf(f,"%g %g %g ", &x_min, &y_min, &z_min );
	fscanf(f,"%g %g %g ", &x_max, &y_max, &z_max );
	x_d = (x_max-x_min)/(x_res-1);
	y_d = (y_max-y_min)/(y_res-1);
	z_d = (z_max-z_min)/(z_res-1);

	grid = new float[x_res*y_res*z_res];
	fread(grid, sizeof(float), x_res*y_res*z_res, f);
	fclose(f);

	truncate_value = *max_element(grid, grid+x_res*y_res*z_res);


	bfs::path file_path(file);
	bfs::path ply_file = bfs::change_extension(file_path, ".ply").leaf();
	bfs::path reduced_path = file_path.parent_path() / "reduced" / ply_file;
	PLYMesh ply_mesh;
	ply_mesh.readFromFile(reduced_path.string(),mesh_);
	mesh_.type = mesh_.MESH;
}


void TemplateModel::show(const ros::Publisher& publisher, const Point32& location, float fit_score)
{
	static int model_id = 0;

	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = "table_frame";
	marker.ns = "voxel_grid";
	marker.id = model_id++;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = location.x;
	marker.pose.position.y = location.y;
	marker.pose.position.z = location.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = x_d;
	marker.scale.y = y_d;
	marker.scale.z = z_d;
	marker.color.a = 0.7;


	if (fit_score<7.0) {
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
	} else {
		return;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}

	for (int i=0;i<x_res;++i) {
		for (int j=0;j<y_res;++j) {
			for (int k=0;k<z_res;++k) {
				if (grid[((i*y_res)+j)*z_res+k]==0) {
					Point p;
					p.x = i*x_d+x_min;
					p.y = j*y_d+y_min;
					p.z = k*z_d+z_min;
					marker.points.push_back(p);
				}
			}
		}
	}
	publisher.publish(marker);
}


void TemplateModel::fitPointCloud(const PointCloud& cloud, const Point32& location, ModelFitSet& mfs)
{
	float score = 0;
	float max_dist = 0;
	for (size_t i=0;i<cloud.pts.size();i++) {
		int x = int(((cloud.pts[i].x-location.x)-x_min)/x_d);
		int y = int(((cloud.pts[i].y-location.y)-y_min)/y_d);
		int z = int(((cloud.pts[i].z-location.z)-z_min)/z_d);

		float val;
		if (in_bounds(x,y,z)) {
			val = grid[((x*y_res)+y)*z_res+z];
		}
		else {
			val = truncate_value;
		}
		max_dist = max(max_dist,val);
		score += val;
	}
	score /= (cloud.pts.size());

	mfs.add(this, location, score, max_dist);
}

void TemplateModel::findBestFit(const PointCloud& cloud, ModelFitSet& mfs)
{
	// compute center of point cloud
	Point32 center;
	center.x =0; center.y = 0; center.z = 0;
	int count = cloud.pts.size();

	for (int i=0;i<count;++i) {
		center.x += cloud.pts[i].x;
		center.y += cloud.pts[i].y;
		//			center.z += cloud.pts[i].z;
	}

	center.x /=count;
	center.y /=count;
	//		center.z /=count;

	Point32 location = center;

	for (float dx=-0.02; dx<=0.02; dx+=0.01) {
		for (float dy=-0.02; dy<=0.02; dy+=0.01) {
			location.x = center.x + dx;
			location.y = center.y + dy;

			fitPointCloud(cloud, location, mfs);
		}
	}
}





class ModelFitter
{
	ros::NodeHandle nh_;
	ros::ServiceServer service_;
	ros::Publisher marker_pub_;

	string template_path;

	vector<TemplateModel*> templates;

#ifdef PUBLISH_GRASH_POSE
	tf::TransformBroadcaster tfb_;
#endif

public:
	ModelFitter()
	{
		nh_.param<string>("~template_path", template_path, "");
		service_ = nh_.advertiseService("recognition_lambertian/model_fit", &ModelFitter::fitModel, this);

		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);

		loadTemplateModels(template_path);
	}

	~ModelFitter()
	{
		for (size_t i=0;i<templates.size();++i) {
			delete templates[i];
		}
	}


	void loadTemplate(const string& filename, const string& model_name)
	{
		ROS_INFO("Loading template: %s", filename.c_str());
		TemplateModel* tpl = new TemplateModel();
		tpl->load(filename, model_name);
		templates.push_back(tpl);
	}

	void loadTemplateModels(const string& path)
	{
		bfs::path template_dir(path);

		if (!bfs::is_directory(template_dir)) {
			ROS_ERROR("Cannot load templates, %s is not a directory", template_dir.leaf().c_str());
		}

		bfs::directory_iterator dir_iter(template_dir), dir_end;
		for(;dir_iter != dir_end; ++dir_iter) {
			if (bfs::extension(*dir_iter)==".dt") {
				loadTemplate(dir_iter->string(), bfs::basename(*dir_iter));
			}
		}

	}


	void fitBestModel(const PointCloud& cloud, ModelFitSet& mfs)
	{
		for (size_t i=0;i<templates.size();++i) {
			templates[i]->findBestFit(cloud, mfs);
		}
	}

	bool fitModel(recognition_lambertian::ModelFit::Request& req,
			recognition_lambertian::ModelFit::Response& resp)
	{
		ROS_INFO("Service called");

		ModelFitSet mfs(1);

		clock_t start = clock();
		fitBestModel(req.cloud, mfs);
		double duration = double(clock()-start)/CLOCKS_PER_SEC;

		ROS_INFO("Best score: %g, time:%g", mfs.best_fit_[0].score(), duration);

		mfs.best_fit_[0].model_->show(marker_pub_,  mfs.best_fit_[0].location_,  mfs.best_fit_[0].score());

		resp.object.grasp_pose.pose = mfs.best_fit_[0].graspPose();
		resp.object.grasp_pose.header.stamp = req.cloud.header.stamp;
		resp.object.grasp_pose.header.frame_id = req.cloud.header.frame_id;
		resp.object.object_pose.pose = mfs.best_fit_[0].objectPose();
		resp.object.object_pose.header.stamp = req.cloud.header.stamp;
		resp.object.object_pose.header.frame_id = req.cloud.header.frame_id;
		resp.object.object = mfs.best_fit_[0].model_->objectMesh();
		resp.score = mfs.best_fit_[0].score();


#ifdef PUBLISH_GRASH_POSE
		tf::Pose tf_pose;
		tf::poseMsgToTF(mfs.best_fit_[0].graspPose(), tf_pose);
		tf::Stamped<tf::Pose> grasp_frame(tf_pose, req.cloud.header.stamp, "grasp_frame", req.cloud.header.frame_id);
		tfb_.sendTransform(grasp_frame);
#endif



		return true;
	}
};


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_fitter");

	model_fit::ModelFitter mf;
	ros::spin();
}
