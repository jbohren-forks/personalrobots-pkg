#include <ros/node.h>
#include "logging/LogPlayer.h"

#include "octree.h"

#include "grasp_learner_types.h"
#include "grasp_learner/OctreeLearningMsg.h"

#include <string>
#include <fstream>
#include <assert.h>

#include "newmat10/newmatap.h"
#include <iomanip>
#include "newmat10/newmatio.h"

#include "lapack_wrappers.h"

/*! \file voxel_processor.cpp

  This executable loads a bag of ROS logs containing
  OctreeLearningMsg. Those have each a voxel grid and a grasp
  point. It then does some sort of processing to them.
*/

using namespace grasp_learner;

template <class T>
void copyMsg(std::string name, ros::msg* m, ros::Time t, void* n);

/*!  Generic class that will parse a ROS bag file and, for each new
  grid that it likes, calls a function that has to be implemented in
  sub-classes. After all the grids have been read from the bag, it
  uses the processData() function to do something on them.
 */
class VoxelProcessor{
protected:
	LogPlayer mLp;
	int mMsgCount;

public:
	virtual ~VoxelProcessor(){}

	bool mNewMsg;
	OctreeLearningMsg mLastMsg;
	OctreeType mOctreeType;
	int mMinOccupiedVoxels;

	virtual bool lastMsgValid() {
		if (mOctreeType != UNSPECIFIED && mOctreeType != mLastMsg.octree_type) return false;
		if (mMinOccupiedVoxels > 0) {
			//we need to count the number of occupied voxels
			scan_utils::Octree<char> octree(0,0,0,0,0,0,0,1);
			octree.setFromMsg( mLastMsg.octree);
			//int empty = octree.cellCount(VOXEL_EMPTY);			
			int occup = octree.cellCount(VOXEL_OCCUPIED);
			/*
			int unknown = octree.cellCount(VOXEL_UNKNOWN);
			fprintf(stderr,"Depth: %d. Voxels: %d empty, %d occupied, %d unknown, %d total\n",
				octree.getMaxDepth(), empty, occup, unknown, empty+occup+unknown);
			*/
			if (occup < mMinOccupiedVoxels) return false;
			
		}
		return true;	
	}
	virtual bool processLastMsg() = 0;
	virtual void processData() = 0;

	virtual bool loadData(std::string filename) {		
		if (!mLp.open(filename, ros::Time(0.0))) return false;
		mLp.addHandler<OctreeLearningMsg>(std::string("grasp_learning_bus"), 
					 &copyMsg<OctreeLearningMsg>, (void*)this, true);
		mNewMsg = false;
		mMsgCount = 0;
		while(mLp.nextMsg()) {
			if (mNewMsg && lastMsgValid() ) {
				if ( processLastMsg() ) mMsgCount++;
			}
			mNewMsg = false;
		}
		mLp.close();
		return true;
	}
	virtual void saveResults(std::string dirname){}
};

template <class T>
void copyMsg(std::string name, ros::msg* m, ros::Time t, void* n)
{
	if (m != 0) {
		((VoxelProcessor*)n)->mLastMsg = *((T*)(m));
		((VoxelProcessor*)n)->mNewMsg = true;
	}
}

/*! Processor that will simply write back to files the octrees found
    in the ROS message bag.
 */
class VoxelWriter : public VoxelProcessor {
public:
	virtual ~VoxelWriter(){}

	virtual bool processLastMsg() {
		char filename[1000];
		scan_utils::Octree<char> octree(0,0,0,0,0,0,0,1);
		std::fstream os;
		
		octree.setFromMsg( mLastMsg.octree);
		sprintf(filename,"logtest/cloud_%d.txt",mMsgCount);
		
		os.open(filename,std::fstream::out);
		if (os.fail()) {
			fprintf(stderr,"Failed to open outfile: %s\n",filename);
			return false;
		} else {
			octree.writeToFile(os);
			os.close();
			fprintf(stderr,"Wrote file: %s\n",filename);
		}
		return true;
	}
	virtual void processData(){
		fprintf(stderr,"Successfully converted %d grids\n",mMsgCount);
	}
};

/*! Just counts and displays the grids in the ROS bag.
 */
class VoxelCounter : public VoxelProcessor {
public:
	virtual ~VoxelCounter(){}

	virtual bool processLastMsg(){return true;}
	virtual void processData(){
		fprintf(stderr,"Read %d grids\n",mMsgCount);
	}	
};

/*! Does Principal Component Analysis on the voxel grids.
 */
class VoxelPCA : public VoxelProcessor {
protected:
	bool mFirstPass;
	int mNumGrids;
	
	float *mGridsMatrix, *mMeansVector;
	float *mEigenValues, *mEigenVectors;
public:
	VoxelPCA() : VoxelProcessor() {mGridsMatrix = NULL; mMeansVector = NULL; mEigenValues = NULL; mEigenVectors = NULL;}
	virtual ~VoxelPCA() {
		if (mGridsMatrix) delete [] mGridsMatrix;
		if (mMeansVector) delete [] mMeansVector;
		if (mEigenValues) delete [] mEigenValues;
		if (mEigenVectors) delete [] mEigenVectors;
	}

	virtual bool loadData(std::string filename) {
		//we first do a pass to count the grids, so we can allocate memory
		mFirstPass = true;
		if (!VoxelProcessor::loadData(filename)) return false;
		mNumGrids = mMsgCount;
		fprintf(stderr,"First data pass; %d grids found.\n",mNumGrids);
		if (!mNumGrids) return 0;
		
		//allocate the matrices
		if (mGridsMatrix) delete mGridsMatrix;
		if (mMeansVector) delete mMeansVector;
		
		mGridsMatrix = new float[8000 * mNumGrids];
		mMeansVector = new float[8000];

		if (!mGridsMatrix || !mMeansVector) {
			fprintf(stderr,"Failed to allocate matrix memory\n");
			return false;
		}

		for (int i=0; i<8000; i++) {
			mMeansVector[i] = 0;
		}
		fprintf(stderr,"Matrices allocated!\n");
		
		//another pass to actually populate the matrix
		mFirstPass = false;
		if (!VoxelProcessor::loadData(filename)) return false;
		fprintf(stderr,"Seconds pass; data loaded.\n");
		return true;
	}

	virtual bool processLastMsg() {
		if (mFirstPass) return true;
		assert( mMsgCount < mNumGrids);

		//read in the octree into a matrix
		scan_utils::Octree<char> octree(0,0,0,0,0,0,0,1);
		octree.setFromMsg( mLastMsg.octree);
		
		if ( octree.getMaxDepth() != 5 ) {
			fprintf(stderr,"Expected octree of depth 5, got depth %d instead\n",
				octree.getMaxDepth() );
			return true;
		}

		//this particular octree that we read in is 32x32x32
		//we only care about the middle 20x20x20 grids
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 20; j++) {
				for(int k = 0; k < 20; k++) {
					int la = 20 * 20 * i + 20 * j + k;
					//re-assign empty and occupied to what we want
					float value = (float)octree.cellGet(i+6, j+6, k+6);

					if (value == (float)VOXEL_EMPTY) {
						value = -1.0;
					} else if (value == (float)VOXEL_OCCUPIED) {
						value = 1.0;
					} else {
						if (value != (float)VOXEL_UNKNOWN) {
							fprintf(stderr,"Unexpected voxel type!\n");
						}
						value = 0.0;
					}
					
					mGridsMatrix[la + mMsgCount * 8000] = value;
					mMeansVector[la] += value;
				}
			}
		}
		
		return true;
	}

	virtual void processData() {
		//sanity check
		fprintf(stderr,"First pass %d and second pass %d\n",mNumGrids, mMsgCount);

		//compute the mean
		float maxMean = 0;
		for (int j=0; j<8000; j++) {
			mMeansVector[j] /= mNumGrids;
			if (mMeansVector[j] > maxMean) maxMean = mMeansVector[j];
		}
		fprintf(stderr,"Largest mean: %f\n",maxMean);

		//normalise the data
		for (int i=0; i<mNumGrids; i++) {
			for (int j=0; j<8000; j++) {
				mGridsMatrix[ j + i*8000 ] -= mMeansVector[j];
			}
		}
		fprintf(stderr,"Data normalized\n");
		
		//compute covariance matrix
		float *covarianceMatrix = new float[8000 * 8000];
		sgemm( "n", "t", //second operand is transposed
		       8000, 8000, mNumGrids, //remember that second operand is transposed
		       1.0 / mNumGrids, mGridsMatrix, 8000,  //first operand
		       mGridsMatrix, 8000, //second operand 
		       0, covarianceMatrix, 8000);
		
		fprintf(stderr,"Covariance matrix computed.\n");
		
		//eigenvalue decomposition
		if (mEigenValues) delete [] mEigenValues;
		mEigenValues = new float[8000];
		float *work = new float[300000];
		int info;
		
		ssyev("V", "U", 
		      8000, covarianceMatrix, 8000,
		      mEigenValues,
		      work, 300000,
		      &info);
		
		fprintf(stderr,"Eigenvalue decomposition done. Info is %d\n",info);
		fprintf(stderr,"Optimal lwork: %f\n",work[0]);

		if (mEigenVectors) delete [] mEigenVectors;
		mEigenVectors = covarianceMatrix;

		delete [] work;
	}

	/*! Saves a vector to a file, both as text and octree of floats */
	virtual void saveVector(float *v, int ldv, int col, std::string name) {
		std::fstream fs;
		std::string filename = name + std::string("_text.txt");
		//save as text
		fs.open(filename.data(), std::fstream::out);
		if (fs.fail()) {
			fprintf(stderr,"Failed to open text file %s\n",filename.data());
			return;
		}

		for (int i=0; i<ldv; i++) {
			fs << v[i + col * ldv] << std::endl;
		}
		fs.close();
		//save as octree
		filename = name + std::string("_octree.txt");
		fs.open(filename.data(),std::fstream::out);
		if(fs.fail()) {
			fprintf(stderr,"Failed to open octree file %s\n",filename.data());
			return;
		}
		scan_utils::Octree<float> octree(0,0,0, 320, 320, 320, 5, (float)VOXEL_UNKNOWN);
		for (int i=0; i<20; i++) {
			for (int j=0; j<20; j++) {
				for (int k=0; k<20; k++) {
					int la = 20*20*i + 20*j + k;
					float value = v[la + col * ldv];
					octree.cellInsert(i+6, j+6, k+6, value);
				}
			}
		}
		octree.writeToFile(fs);
		fs.close();					
	}

	virtual void saveResults(std::string dirname) {
		std::fstream fs;
		if (!mEigenValues || !mEigenVectors || !mMeansVector) {
			fprintf(stderr,"Unable to save results: data missing!\n");
			//return;
			mMeansVector = new float[8000];
			mEigenValues = new float[8000];
			mEigenVectors = new float[8000 * 8000];
		}

		//compute some statistics
		float totalVariance = 0;
		for (int i=0; i<8000; i++) {
			totalVariance += mEigenValues[i];
		}


		float currentVariance = 0;
		/*
		for (int i=1; i<50; i++) {
			currentVariance += mEigenValues[ 8000 - i ];
			fprintf(stderr,"Variance up to %d dimensions: %f\n",i,currentVariance / totalVariance);
		}	       
		*/
		//save all eigenvalues  along with some stats. Remember they are stored IN ASCENDING ORDER!
		std::string egfile = dirname + std::string("/eigenvalues.txt");
		fs.open(egfile.data(), std::fstream::out);
		if (fs.fail()) {
			fprintf(stderr,"Failed to open eg file %s\n",egfile.data());
			return;
		}
		currentVariance = 0;
		for (int i=1; i<=8000; i++) {
			float ev = mEigenValues[8000-i];
			currentVariance += ev;
			fs << i-1 << " " << ev << " " << ev/totalVariance << " " << currentVariance/totalVariance << std::endl;
		}
		fs.close();

		//save the mean
		egfile = dirname + std::string("/mean");
		saveVector(mMeansVector, 8000, 0, egfile);

		//save first 100 eigenvectors
		char num[50];
		for (int ev=1; ev<=100; ev++) {
			sprintf(num,"/eigenvector_%d",ev-1);
			egfile = dirname + std::string(num);
			saveVector(mEigenVectors, 8000, 8000-ev, egfile);
		}
	}
};

void usage() 
{
	fprintf(stderr,"Usage: voxel_processor type input_file output_directory voxel_types [min_voxels]\n");
}

int main(int argc, char **argv)
{
	if (argc < 5) {
		usage();
		return 0;
	}

	VoxelProcessor *vp;
	if (!strcmp(argv[1],"writer")) {
		vp = new VoxelWriter;
	} else if (!strcmp(argv[1],"counter")) {
		vp = new VoxelCounter;
	} else if (!strcmp(argv[1],"pca")) {
		vp = new VoxelPCA;
	} else {
		fprintf(stderr,"Can not understand processor type: %s\n",argv[1]);
		return 0;
	}

	std::string fullname=argv[2];
	std::string outname = argv[3];

	OctreeType type;
	if (!strcmp(argv[4],"BINARY_FREE")) {
		type = BINARY_FREE;
	} else if (!strcmp(argv[4],"BINARY_ALIGNED")) {
		type = BINARY_ALIGNED;
	} else if (!strcmp(argv[4],"CARVED_FREE")) {
		type = CARVED_FREE;
	} else if (!strcmp(argv[4],"CARVED_ALIGNED")) {
		type = CARVED_ALIGNED;
	} else if (!strcmp(argv[4],"ALL")) {
		type = UNSPECIFIED;
	} else{
		fprintf(stderr,"Can not understand type of voxel grids to process: %s\n",argv[4]);
		return 0;
	}
	vp->mOctreeType = type;

	int minVox = -1;
	if (argc >=6) {
		minVox = atoi(argv[5]);
	}
	if (minVox <= 0) {
		fprintf(stderr,"Processing all grids regardless of number of voxels\n");
		minVox = -1;
	} else {
		fprintf(stderr,"Only processing grids with at least %d voxels\n",minVox);
	}
	vp->mMinOccupiedVoxels = minVox;

	vp->loadData(fullname);
	vp->processData();
	vp->saveResults(outname);

	delete vp;
}
