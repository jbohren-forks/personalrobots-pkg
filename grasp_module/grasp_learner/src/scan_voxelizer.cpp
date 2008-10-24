#include <ros/node.h>
#include "libTF/libTF.h"
#include "smartScan.h"
#include "graspPoint.h"
#include "octree.h"

#include "grasp_learner_types.h"
#include "grasp_learner/OctreeLearningMsg.h"
#include "std_msgs/Point32.h"

#include <fstream>
#include <vector>
#include <list>
#include <errno.h>
#include <unistd.h>

/**
   @mainpage @b A collection of tools for reading in, parsing,
   processing and saving data from the Grasp Learning Database.

   The scan_voxelizer reads in files containing point clouds and
   associated grasp points and voxelizes them, then saves them as
   files and also broadcasts them as ROS messages.

   The voxel_processor reads in voxels from a ROS bag and does
   processing on them, such as Principal Component Analysis.

**/

using namespace grasp_module;
using namespace scan_utils;
using namespace grasp_learner;

void transformPoint(float *t, float ox, float oy, float oz, float &tx, float &ty, float &tz)
{
	tx = ox * t[0] + oy * t[1] + oz * t[2] + 1 * t[3];
	ty = ox * t[4] + oy * t[5] + oz * t[6] + 1 * t[7];
	tz = ox * t[8] + oy * t[9] + oz * t[10] + 1 * t[11];
}

void rotatePoint(float *t, float ox, float oy, float oz, float &tx, float &ty, float &tz)
{
	tx = ox * t[0] + oy * t[1] + oz * t[2];
	ty = ox * t[4] + oy * t[5] + oz * t[6];
	tz = ox * t[8] + oy * t[9] + oz * t[10];
}

class MessageSender : public ros::node
{
public:
	MessageSender() : ros::node("grasp_learning_node"){
		advertise<OctreeLearningMsg>("grasp_learning_bus");
	}
	template <typename T>
	void sendMessage(Octree<T> &octree, GraspPoint *grasp, OctreeType type) {
		OctreeLearningMsg msg;
		octree.getAsMsg(msg.octree);
		msg.octree_type = type;
		msg.grasp_point = grasp->getMsg();
		//Octrees are always centered at origin of grasp point
		msg.grasp_point.frame.xt = 0;
		msg.grasp_point.frame.yt = 0;
		msg.grasp_point.frame.zt = 0;
		if (type == BINARY_ALIGNED || type == CARVED_ALIGNED) {
			//we have also aligned the axes
			msg.grasp_point.frame.xr = 0;
			msg.grasp_point.frame.yr = 0;
			msg.grasp_point.frame.zr = 0;
			msg.grasp_point.frame.w = 1;			
		}
		publish("grasp_learning_bus",msg);
	}
};


/*!  This utility function seeks a line in the stream with contents
  identical to the string passed as argument. If the string is found,
  it returns true and leaves the stream positioned at the beginning of
  the line following the string. If it is not found, it returns false
  and leaves the string positioned at eof.
 */
bool findString(std::istream *fs, const char * string)
{
	char line[1000];
	while (!fs->eof()) {
		fs->getline(line, 1000);
		if (!strcmp(string,line)) return true;
	}
	return false;
}

/*!  Reads in a grasp point for the PR2 gripper from a stream, in the
  particular (and also rather peculiar) style that GraspIt! saves them in.
 */
GraspPoint *readGraspPoint(fstream &fs)
{
	int foo;
	float dofVal;
	//first line starts with an int that we ignore, then it's the value of the DOF
	fs >> foo >> dofVal;
	if (fs.fail()) return NULL;
	float tx,ty,tz,qw,qx,qy,qz;
	//the next line also starts with an int we ignore, then it has the transform 
	fs >> foo >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
	if (fs.fail()) return NULL;
	float qual;
	//the first thing on the next line is the quality of the grasp
	fs >> qual;
	if (fs.fail()) return NULL;
	GraspPoint *ret = new GraspPoint();

	ret->setTran(tx,ty,tz,
		     qx,qy,qz,qw);
	ret->setQuality(qual);
	return ret;
}

template <typename T>
bool writeOctreeToFile(scan_utils::Octree<T> &octree, char *outfile)
{
	std::fstream os;
	os.open(outfile,std::fstream::out);
	if (os.fail()) {
		fprintf(stderr,"Failed to open outfile: %s\n",outfile);
		return false;
	} else {
		octree.writeToFile(os);
		os.close();
		return true;
	}
}

struct Ray{
	float dx, dy, dz;
	float dist;
};

/*!  This class loads a file saved by GraspIt! containing a point
  cloud and a number of grasp points. It then creates voxel grids
  around each grasp point and saves them as files and also broadcasts
  them as ROS messages that can be logged.
 */

class ScanVoxelizer
{
public:
	enum Type {CLOUD, RAW};
private:
	char mInfile[1000], mOutdir[1000], mOutfile[100];
	int mMinPoints, mFileCount;
	Type mType;

	//for saving clouds
	SmartScan mCloud;

	//for saving raw data
	//ray direction
	std::list<Ray> mRays;
	//scanner position and up direction
	float mSx, mSy, mSz;
	float mUx, mUy, mUz;

	std::vector<GraspPoint*> mGrasps;
     
	void usage();
public:
	ScanVoxelizer(){mSender = NULL;}
	MessageSender *mSender;
	bool parseArgs(int argc, char **argv);
	bool loadData();
	bool loadRawFile(std::fstream &fs);
	bool processCloudAndGrasp(SmartScan *cloud, GraspPoint *newGrasp);
	bool processRaysAndGrasp(std::list<Ray> &rays, GraspPoint *newGrasp);
	void processData();
};

bool ScanVoxelizer::parseArgs(int argc, char **argv)
{
	if (argc < 5) {
		usage();
		return false;
	}

	strcpy(mInfile, argv[1]);

	if ( !strcmp(argv[2],"cloud") ) {
		mType = CLOUD;
	} else if ( !strcmp(argv[2],"raw") ) {
		mType = RAW;
	} else {
		fprintf(stderr,"Failed to understand type parameter %s, should be 'cloud' or 'raw'\n",argv[2]);
		return false;
	}

	strcpy(mOutdir, argv[3]);
	if (mOutdir[strlen(mOutdir)-1] != '/') {
		strcat(mOutdir,"/");
	}
	strcpy(mOutfile, argv[4]);

	//min number of voxels for saving
	errno = 0;
	mMinPoints = strtol(argv[5], (char**)NULL, 10);
	if (mMinPoints <=0 || errno != 0) {
		fprintf(stderr,"Can't read min voxels: %s\n",argv[5]);
		return false;
	}
	return true;
}

void ScanVoxelizer::usage()
{
	fprintf(stderr,"Usage: voxelizer filename type dest_directory dest_base_name min_voxels\n");
}

bool ScanVoxelizer::loadRawFile(std::fstream &fs)
{
	char line[1000];
	int numRays, readRays = 0;
	bool done = false;
	Ray ray;
	float foo;

	//first comes the scanner location
	fs.getline(line,1000);
	if (sscanf(line,"%f %f %f",&mSx, &mSy, &mSz) != 3) {
		fprintf(stderr,"Failed to read scanner position in line: %s\n",line);
		return false;
	}
	//scanner direction and up direction
	fs.getline(line,1000);
	fs.getline(line,1000);
	if (sscanf(line,"%f %f %f",&mUx, &mUy, &mUz) != 3) {
		fprintf(stderr,"Failed to read scanner up direction in line: %s\n",line);
		return false;
	}
	//then comes the number of rays on its own line
	fs >> numRays; fs.getline(line,1000);
	if (numRays <=0) {
		fprintf(stderr,"Failed to read numRays: %d\n",numRays);
		return false;
	}
	
	//read the rays
	while (readRays < numRays && !done){
		fs.getline(line,1000);
		if ( sscanf(line,"%f %f %f %f %f %f",&foo,&foo,&ray.dx,&ray.dy, &ray.dz, &ray.dist) != 6 ) {
			fprintf(stderr,"Failed to read ray in line: %s\n",line);
			done = true;
		} else {
			readRays++;
			mRays.push_back(ray);
		}
	}
	if (!readRays || readRays != numRays) return false;
	fprintf(stderr,"Read %d rays\n",numRays);
	return true;
}

bool ScanVoxelizer::loadData()
{
	std::fstream fs;
	fs.open(mInfile,std::fstream::in);
	if (fs.fail()){
		fprintf(stderr,"Failed to open file %s\n",mInfile);
		return false;
	}

	if (mType == CLOUD) {
		if (!mCloud.readFromFile(fs)) {
			fprintf(stderr,"Failed to read scan from file %s\n",mInfile);
			fs.close();
			return false;
		}
	} else if (mType == RAW) {
		if (!loadRawFile(fs)) {
			fprintf(stderr,"Failed to read raw data from file %s\n",mInfile);
			fs.close();
			return false;
		}
	}

	GraspPoint *newGrasp;
	//read in grasp points in the particular format that GraspIt! saved them in
	//all grasp points are preceded by the keyword "pre-grasp"
	fs.seekg(std::ios::beg);
	while (findString(&fs,"pre-grasp")) {
		newGrasp = readGraspPoint(fs);
		if (newGrasp) mGrasps.push_back(newGrasp);
	}

	fs.close();
	if ( mGrasps.empty() ) {
		fprintf(stderr,"No grasps read from file\n");
		return false;
	}

	return true;
}

bool ScanVoxelizer::processCloudAndGrasp(SmartScan *cloud, GraspPoint *newGrasp)
{
	char outfile[1000], outNum[50];
	float *transform;

	//initialize an octree
	//remember that GraspIt works in mm!
	Octree<char> octree(0,0,0,
			    320, 320, 320, //20 cm size in each direction
			    5, VOXEL_UNKNOWN); //depth 5, 1cm cells
	SmartScan savedCloud;	

	savedCloud.addScan(cloud);
	
	libTF::TFPoint pt = newGrasp->getLocation();
	newGrasp->getTranInv(&transform);
	
	//first save unaligned octree, just centered at the grasp points
	octree.setCenter(pt.x, pt.y, pt.z);
	octree.clear();
	savedCloud.insertInOctree(&octree, VOXEL_OCCUPIED);
	//don't bother with really empty octrees
	if (octree.getNumLeaves() < mMinPoints) {
		delete [] transform;
		return false;
	}
	
	//tell the octree it's centered at 0,0,0 for saving. This does not change the inner structure
	octree.setCenter(0.0, 0.0, 0.0);
	
	//put together the filename and save Octree to file
	strcpy(outfile, mOutdir);
	strcat(outfile, mOutfile);
	sprintf(outNum,"binary_free_%d.txt",mFileCount);
	strcat(outfile, outNum);
	
	//save octree both as file and ROS message
	writeOctreeToFile<char>(octree, outfile);
	if (mSender) mSender->sendMessage(octree,newGrasp,BINARY_FREE);		
	
	//save cloud that is also rotated to be aligned with grasp direction
	savedCloud.applyTransform(transform);
	octree.clear();
	//the cloud should be already set with origin at grasp point 
	octree.setCenter(0.0, 0.0, 0.0);
	savedCloud.insertInOctree(&octree, VOXEL_OCCUPIED);
	
	//put together the filename and save Octree to file
	strcpy(outfile, mOutdir);
	strcat(outfile, mOutfile);
	sprintf(outNum,"binary_aligned_%d.txt",mFileCount);
	strcat(outfile, outNum);
	
	//save octree both as file and ROS message
	writeOctreeToFile<char>(octree, outfile);
	if (mSender) mSender->sendMessage(octree,newGrasp,BINARY_ALIGNED);
	delete [] transform;
	return true;
}
 
bool ScanVoxelizer::processRaysAndGrasp(std::list<Ray> &rays, GraspPoint *newGrasp)
{
	char outfile[1000], outNum[50];
	float *transform;
	
	//initialize an octree
	//remember that GraspIt works in mm!
	Octree<char> octree(0,0,0,
			    320, 320, 320, //32 cm size in each direction
			    5, VOXEL_UNKNOWN); //depth 5, 1 cm cells

	libTF::TFPoint pt = newGrasp->getLocation();
	newGrasp->getTranInv(&transform);
	
	//first save unaligned octree, just centered at the grasp points
	octree.setCenter(pt.x, pt.y, pt.z);

	std::list<Ray>::iterator it;
	//trace the rays
	for (it = rays.begin(); it!=rays.end(); it++) {
		octree.traceRay(mSx, mSy, mSz, 
				(*it).dx, (*it).dy, (*it).dz, (*it).dist,
				VOXEL_EMPTY, VOXEL_OCCUPIED);
	}
	//re-insert the occupied voxels so they don't get erased by rays
	for (it = rays.begin(); it!=rays.end(); it++) {
		if ( (*it).dist < 0 ) continue;
		octree.insert(mSx + (*it).dist * (*it).dx, 
			      mSy + (*it).dist * (*it).dy, 
			      mSz + (*it).dist * (*it).dz, VOXEL_OCCUPIED);
	}

	//don't bother with really empty octrees
	if (octree.getNumLeaves() < mMinPoints) {
		delete [] transform;
		return false;
	}
	//tell the octree it's centered at 0,0,0 for saving. This does not change the inner structure
	octree.setCenter(0.0, 0.0, 0.0);
	
	//put together the filename and save Octree to file
	strcpy(outfile, mOutdir);
	strcat(outfile, mOutfile);
	sprintf(outNum,"carved_free_%d.txt",mFileCount);
	strcat(outfile, outNum);
	
	//save octree both as file and ROS message
	writeOctreeToFile<char>(octree, outfile);
	if (mSender) mSender->sendMessage(octree,newGrasp,CARVED_FREE);		
	
	//save cloud that is also rotated to be aligned with grasp direction
	octree.clear();
	//the cloud should be already set with origin at grasp point 
	octree.setCenter(0.0, 0.0, 0.0);

	SmartScan bogusScan;
	float sx, sy, sz, dx, dy, dz;

	transformPoint(transform, mSx, mSy, mSz, sx, sy, sz);
	for (it = rays.begin(); it!=rays.end(); it++) {
		rotatePoint(transform,(*it).dx, (*it).dy, (*it).dz, dx, dy, dz );
		octree.traceRay(sx, sy, sz, 
				dx, dy, dz, (*it).dist,
				VOXEL_EMPTY, VOXEL_OCCUPIED);
	}
	//re-insert the occupied voxels so they don't get erased by rays
	for (it = rays.begin(); it!=rays.end(); it++) {
		if ( (*it).dist < 0 ) continue;
		rotatePoint(transform,(*it).dx, (*it).dy, (*it).dz, dx, dy, dz );
		octree.insert(sx + (*it).dist * dx, 
			      sy + (*it).dist * dy, 
			      sz + (*it).dist * dz, VOXEL_OCCUPIED);
	}
	
	//put together the filename and save Octree to file
	strcpy(outfile, mOutdir);
	strcat(outfile, mOutfile);
	sprintf(outNum,"carved_aligned_%d.txt",mFileCount);
	strcat(outfile, outNum);
	
	//save octree both as file and ROS message
	writeOctreeToFile<char>(octree, outfile);
	if (mSender) mSender->sendMessage(octree,newGrasp,CARVED_ALIGNED);
	
	fprintf(stderr,"File: %s\n",outfile);
	
	delete [] transform;
	return true;
}

void ScanVoxelizer::processData()
{
	mFileCount = 0;
	while (!mGrasps.empty()){
		GraspPoint *newGrasp = mGrasps.back();
		mGrasps.pop_back();
		if (mType == CLOUD) {
			if (processCloudAndGrasp(&mCloud, newGrasp)) {
				mFileCount ++;
			}
			//cloud messages are sent faster than ROS vacuum can keep up with
			usleep(10000);
		} else if (mType == RAW) {
			if (processRaysAndGrasp(mRays, newGrasp)){
				mFileCount++;
			}
		}
		delete newGrasp;
	}
	fprintf(stderr,"Finished. File count: %d\n",mFileCount);
}

int main(int argc, char **argv)
{	
	ScanVoxelizer mVox;
	if ( !mVox.parseArgs(argc, argv) ) return 0;
	if ( !mVox.loadData() ) return 0;

	//prepare ROS sender
	ros::init(argc, argv);
	MessageSender sender;
	mVox.mSender = &sender;

	mVox.processData();

	ros::fini();
}

