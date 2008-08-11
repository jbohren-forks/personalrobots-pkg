#include "octree.h"
#include <gtest/gtest.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream>
#include <scan_utils/OctreeMsg.h>

/*! \file Since the Octree is templated, it is completely contained in
  header files. As a result it is no longer compiled into the
  scan_utils .so. This file creates an executable that is just meant
  to make the library actually compile the Octree header files to make
  sure they work. Can be considered a (fairly simple) unit test for
  the octree class.
*/

TEST(OctreeTests, constructionDeconstruction)
{
	scan_utils::Octree<int> *octree = new scan_utils::Octree<int>(0,0,0, 1,1,1, 1, 0);
	delete octree;
}

TEST(OctreeTests, insertionExtraction)
{
	scan_utils::Octree<float> *octree = new scan_utils::Octree<float>(0,0,0, 1.0,2.0,2.0, 5, 0.2);
	srand( (unsigned)time(NULL) );
	bool pass = true;
	float compVal;
	for (int i=0; i<100; i++) {
		float x,y,z;
		x = ((float)rand()) / (RAND_MAX + 1);
		y = ((float)rand()) / (RAND_MAX + 1);
		z = ((float)rand()) / (RAND_MAX + 1);
		if (!octree->testBounds(x,y,z)) {
			compVal = 0.2;
		} else {
			compVal = 1.7;
		}
		octree->insert(x, y, z, 1.7);

		float r = octree->get(x,y,z);
		if ( fabs(r - compVal) > 1.0e-6 ) {
			pass = false;
		}
	}
	EXPECT_TRUE(pass);
	delete octree;
}

TEST (OctreeTests, serializationDeserialization)
{
	int depth = 4;
	int numCells = (int)pow(2,depth);
	float cellSize = 1.0 / numCells;

	scan_utils::Octree<float> *original = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, 0.2);
	srand( (unsigned)time(NULL) );
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				original->insert(i1,i2,i3,((float)rand()) / RAND_MAX);
			}
		}
	}

	std::fstream os;
	os.open("testOctree.dat",std::fstream::out);
	original->writeToFile(os);
	os.close();

	scan_utils::Octree<float> *copy = new scan_utils::Octree<float>(0.4,0.4,0.1, 1.5,1.7,1.1, depth, 0.5);
	
	std::fstream is;
	is.open("testOctree.dat",std::fstream::in);
	copy->readFromFile(is);
	is.close();

	bool pass = true;
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				float c = copy->get(i1,i2,i3);
				float o = original->get(i1,i2,i3);
				if ( fabs( c - o ) > 1.0e-6 ) {
					pass = false;
					//fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
			}
		}
	}
	EXPECT_TRUE(pass);
	
	delete original;
	delete copy;
}

TEST (OctreeTests, toFromROSMessage)
{
	int depth = 4;
	int numCells = (int)pow(2,depth);
	float cellSize = 1.0 / numCells;

	scan_utils::Octree<float> *original = new scan_utils::Octree<float>(0,0,0, 1.0,1.0,1.0, depth, 0.2);
	srand( (unsigned)time(NULL) );
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				original->insert(i1,i2,i3,((float)rand()) / RAND_MAX);
			}
		}
	}

	scan_utils::OctreeMsg msg;
	original->getAsMsg(msg);

	scan_utils::Octree<float> *copy = new scan_utils::Octree<float>(0.4,0.4,0.1, 1.5,1.7,1.1, depth, 0.5);
	copy->setFromMsg(msg);
	
	bool pass = true;
	for (float i1 = 0; i1 < numCells * cellSize; i1+=cellSize) {
		for (float i2 = 0; i2 < numCells * cellSize; i2+=cellSize) {
			for (float i3 = 0; i3 < numCells * cellSize; i3+=cellSize) {
				float c = copy->get(i1,i2,i3);
				float o = original->get(i1,i2,i3);
				if ( fabs( c - o ) > 1.0e-6 ) {
					pass = false;
					//fprintf(stderr,"Original %f and copy %f \n",o,c);
				}
			}
		}
	}
	EXPECT_TRUE(pass);
	
	delete original;
	delete copy;
}


int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
