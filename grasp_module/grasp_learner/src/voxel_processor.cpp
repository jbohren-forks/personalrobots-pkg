#include <ros/node.h>
#include "logging/LogPlayer.h"

#include "octree.h"

#include "grasp_learner_types.h"
#include "grasp_learner/OctreeLearningMsg.h"

#include <string>
#include <fstream>

/*! \file voxel_processor.cpp

  This executable loads a bag of ROS logs containing
  OctreeLearningMsg. Thos have each a voxel grid and a grasp
  point. It then does some sort of processing to them.
*/

using namespace grasp_learner;

template <class T>
void copyMsg(std::string name, ros::msg* m, ros::Time t, void* n);

class VoxelProcessor{
private:
	LogPlayer mLp;
	int mMsgCount;

public:
	bool mNewMsg;
	OctreeLearningMsg mLastMsg;
	void processLastMsg();
	bool loadData(std::string filename) {
		
		mLp.open(filename, ros::Time(0));
		mLp.addHandler<OctreeLearningMsg>(std::string("grasp_learning_bus"), 
					 &copyMsg<OctreeLearningMsg>, (void*)this, true);
		mNewMsg = false;
		mMsgCount = 0;
		while(mLp.nextMsg()) {
			if (mNewMsg) {
				processLastMsg();
				mMsgCount++;
			}
			mNewMsg = false;
		}
		
		return true;
	}
};

template <class T>
void copyMsg(std::string name, ros::msg* m, ros::Time t, void* n)
{
	if (m != 0) {
		((VoxelProcessor*)n)->mLastMsg = *((T*)(m));
		((VoxelProcessor*)n)->mNewMsg = true;
	}
}


void VoxelProcessor::processLastMsg()
{
	char filename[1000];
	char num[50];
	scan_utils::Octree<char> octree(0,0,0,0,0,0,0,1);
	std::fstream os;

	octree.setFromMsg( mLastMsg.octree);
	if (mLastMsg.octree_type == BINARY_FREE) {
		strcpy(filename,"logtest/cloud__bf_");
	} else if (mLastMsg.octree_type == BINARY_ALIGNED) {
		strcpy(filename,"logtest/cloud_ba_");
	} else if (mLastMsg.octree_type == CARVED_FREE) {
		strcpy(filename,"logtest/cloud_cf_");
	} else if (mLastMsg.octree_type == CARVED_ALIGNED) {
		strcpy(filename,"logtest/cloud_ca_");
	}

	sprintf(num,"%d.txt",mMsgCount);
	strcat(filename,num);
	
	os.open(filename,std::fstream::out);
	if (os.fail()) {
		fprintf(stderr,"Failed to open outfile: %s\n",filename);
	} else {
		octree.writeToFile(os);
		os.close();
		fprintf(stderr,"Wrote file: %s\n",filename);
	}	      
}


void usage() 
{
	fprintf(stderr,"Usage: voxel_processor filename\n");
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		usage();
		return 0;
	}
	std::string fullname=argv[1];

	VoxelProcessor vp;
	vp.loadData(fullname);
}
