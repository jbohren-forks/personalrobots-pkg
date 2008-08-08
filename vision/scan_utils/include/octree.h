#ifndef _octree_h_
#define _octree_h_

#include <stdlib.h>
#include <list>
#include <dataTypes.h>
#include <scan_utils/OctreeMsg.h>

namespace scan_utils {

/*!  A generic Octree node, could either be a leaf or a branch. Could
  be defined privately inside the Octree class, but in the future we
  might implement faster accessors that return the node that they
  accessed, so that subsequent calls can use this information.
*/
class OctreeNode {
 private:
 public:
	OctreeNode(){}
	virtual bool isLeaf() = 0;
	virtual void getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				  float cx, float cy, float cz,
				  float dx, float dy, float dz,
				  std::list<Triangle> &triangles) = 0;
	virtual void serialize(char *destinationString, unsigned int &address){}
	virtual void deserialize(char *sourceString, unsigned int &address, unsigned int size){}
	virtual int computeMaxDepth(){return 0;}
};

/*! An Octree branch. Always contains exactly 8 children pointers. A
    NULL child pointer means that the respective child points to an
    unexplored region of space and thus is equivalent to having a
    child with the mEmptyValue of the Octree set.
 */
class OctreeBranch : public OctreeNode {
 private:
	OctreeNode **mChildren;
 public:
	bool isLeaf(){return false;}

	//! Initializes a branch with all NULL (unexplored) children
	inline OctreeBranch();
	//! Initializes a branch with all children set to the value \a val 
	inline OctreeBranch(int val);
	//! Destructor will delete all children first. Thus, delete an Octree top-down by just deleting its root
	inline ~OctreeBranch();

	//! Return the child at address \a adress, between 0 and 7
	OctreeNode* getChild(unsigned char address) { return mChildren[address]; }
	//! Sets the child at address \a adress to point at \a child. 
	/*! If a child was already present at that address it is deleted.*/
	inline void setChild(unsigned char address, OctreeNode *child);
	//! Recursively returns the total number of branches below this one (including this one)
	int getNumBranches();
	//! Recursively returns the number of leaves between this one
	int getNumLeaves();
	//! Recursively return the triangles that form the surface of non-empty cells below this branch, 
	virtual void getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				  float cx, float cy, float cz,
				  float dx, float dy, float dz,
				  std::list<Triangle> &triangles);
	//! Recursively serializes everything below this branch
	virtual void serialize(char *destinationString, unsigned int &address);
	//! Recursively reads in everything below this branch
	virtual void deserialize(char *sourceString, unsigned int &address, unsigned int size);
	//! Recursively computes the max depth under this branch
	virtual int computeMaxDepth();

};

/*! A leaf simply holds a value and nothing else. Do not use a leaf to
    store the empty value, use a NULL pointer in its parent instead.
 */
class OctreeLeaf : public OctreeNode {
 private:
	int mValue;
 public:
        OctreeLeaf(int val) : mValue(val) {}
	OctreeLeaf(){}
	~OctreeLeaf(){}
	bool isLeaf(){return true;}

	int getVal(){return mValue;}
	void setVal(int val){mValue = val;}
	//! Returns the triangles that surround this cell. 
	virtual void getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				  float cx, float cy, float cz,
				  float dx, float dy, float dz,
				  std::list<Triangle> &triangles);
	// Serializes the content of this leaf
	virtual void serialize(char *destinationString, unsigned int &address);
	// Reads in the content of this leaf
	virtual void deserialize(char *sourceString, unsigned int &address, unsigned int size);
	//! Returns 0
	virtual int computeMaxDepth(){return 0;}

};

/*! An Octree class. It is designed for access based on spatial
    coordinates, rather than cell indices.

    The Octree needs to know what part of the 3D world it is
    responsible for. It has a defined center somewhere in space and
    defined dimensions. After this is defined, just use spatial
    coordinates for insertions / queries and you never have to worry
    about cell indices.

    You can specifiy different extents along x,y and z. However, for
    now, this just means that all the Octree branches and leaves will
    have that given aspect ration. It is not smart enough to always
    have cubic leaves, but more of them along one direction than
    another.

    You can specify the maximum depth of the tree as well. A \a
    maxDepth of \d guarantees that the smallest leaves will have size
    2^(-d) * total_octree_size.

    Example: an octree that covers a space of 20m * 20m * 20m with max
    depth 10 will have the smallest cells of approx. 2cm * 2cm * 2cm
    in size.

    Set an empty value that will be returned for all unvisited regions
    of space. If you query the value of a region in space that you
    never set, the empty value will be returned.
*/
class Octree {
 private:
	//! The root of the Octree - always a branch and never NULL.
	OctreeBranch *mRoot;
	//! The max depth of the octree, minimum 1 (root and 8 leaves).
	/*! The smallest cell is guaranteed to have size of 2^(-maxDepth) * total_octree_size
	 */
	int mMaxDepth;
	//! The total size of the Octree. Can be thought of as the dimensions of the root cell.
	float mDx, mDy, mDz;
	//! The location of the center of the octree.
	float mCx, mCy, mCz;
	//! The value that is returned for a never visited cell
	int mEmptyValue;
 public:
	//! Constructor needs all initialization values.
	Octree(float cx, float cy, float cz,
	       float dx, float dy, float dz, 
	       int maxDepth, int emptyValue);
	//! Recursively deletes the tree by deleting the root.
	~Octree(){delete mRoot;}
	//! Sets the center of this Octree. Does NOT change the inner data.
	void setCenter(float cx, float cy, float cz){mCx = cx; mCy = cy; mCz = cz;}
	//! Sets the size of this Octree. Does NOT change the inner data.
	void setSize(float dx, float dy, float dz){mDx = dx; mDy = dy; mDz = dz;}
	//! Sets the max depth of this Octree. Does NOT change the inner data.
	void setDepth(int d){if (d<=0) d=1; mMaxDepth = d;}
	//! Returns \a true if the point at x,y,z is inside the volume of this Octree
	inline bool testBounds(float x, float y, float z);
	//! Inserts a value at given spatial coordinates.
	void insert(float x, float y, float z, int newValue);
	//! Returns the value at given spatial coordinates
	int get(float x, float y, float z);
	//! Clears and deallocates the entire Octree. 
	void clear();

	//! Returns the triangles that form the surface mesh of the non-empty cells
	void getTriangles(std::list<Triangle> &triangles);
	//! Recursively computes and returns the number of branches of the tree
	int getNumBranches(){return mRoot->getNumBranches();}
	//! Recursively computes and returns the number of leaves of the tree
	int getNumLeaves(){return mRoot->getNumLeaves();}
	//! Returns the space in memory occupied by the tree
	long long unsigned int getMemorySize();

	//! Serializes this octree to a string
	void serialize(char **destinationString, unsigned int *size);
	//! Reads in the content of this Octree. OLD CONTENT IS DELETED!
	bool deserialize(char *sourceString, unsigned int size);

	//! Constants that need to be chars to save space
	static const char NULL_CHILD, BRANCH, LEAF;

	//! Recursively computes the max depth in the Octree. Useful when Octree is read from file.
	virtual int computeMaxDepth(){return mRoot->computeMaxDepth();}
	//! Set this Octree from a ROS message
	void setFromMsg(const OctreeMsg &msg);
	//! Write this Octree in a ROS message
	void getAsMsg(OctreeMsg &msg);
	//! Write the Octree to a file in its own internal format
	void writeToFile(std::ostream &os);
	//! Read Octree from a file saved by the \a writeToFile(...) function 
	void readFromFile(std::istream &is);
};

OctreeBranch::OctreeBranch()
{
	mChildren = new OctreeNode*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = NULL;
	}
}	

OctreeBranch::OctreeBranch(int val)
{
	mChildren = new OctreeNode*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = new OctreeLeaf(val);
	}
}	

OctreeBranch::~OctreeBranch()
{
	for(int i=0; i<8; i++) {
		if (mChildren[i]) delete mChildren[i];
	}
	delete mChildren;
}
 
void OctreeBranch::setChild(unsigned char address, OctreeNode *child) 
{
	if (mChildren[address]) delete mChildren[address];
	mChildren[address] = child; 
}

bool Octree::testBounds(float x, float y, float z)
{
	float dx = mDx / 2.0;
	float dy = mDy / 2.0;
	float dz = mDz / 2.0;
	if ( x > mCx + dx ) return false; if (x < mCx - dx ) return false;
	if ( y > mCy + dy ) return false; if (y < mCy - dy ) return false;
	if ( z > mCz + dz ) return false; if (z < mCz - dz ) return false;
	return true;
}


} //namespace scan_utils

#endif
