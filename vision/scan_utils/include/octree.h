#ifndef _octree_h_
#define _octree_h_

#include "octreeNodes.h"
#include <scan_utils/OctreeMsg.h>

namespace scan_utils {


/*! An Octree class. It is designed for access based on spatial
    coordinates, rather than cell indices.

    The datatype that is held in the leaves is templated. For now, the
    only requirement on the type is that is must allow the assignment
    operator =. 

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
template <typename T>
class Octree {
 private:
	//! The root of the Octree - always a branch and never NULL.
	OctreeBranch<T> *mRoot;
	//! The max depth of the octree, minimum 1 (root and 8 leaves).
	/*! The smallest cell is guaranteed to have size of 2^(-maxDepth) * total_octree_size
	 */
	int mMaxDepth;
	//! The total size of the Octree. Can be thought of as the dimensions of the root cell.
	float mDx, mDy, mDz;
	//! The location of the center of the octree.
	float mCx, mCy, mCz;
	//! The value that is returned for a never visited cell
	T mEmptyValue;
 public:
	//! Constructor needs all initialization values.
	Octree(float cx, float cy, float cz,
	       float dx, float dy, float dz, 
	       int maxDepth, T emptyValue);
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
	void insert(float x, float y, float z, T newValue);
	//! Returns the value at given spatial coordinates
	T get(float x, float y, float z);
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

template <typename T>
bool Octree<T>::testBounds(float x, float y, float z)
{
	float dx = mDx / 2.0;
	float dy = mDy / 2.0;
	float dz = mDz / 2.0;
	if ( x > mCx + dx ) return false; if (x < mCx - dx ) return false;
	if ( y > mCy + dy ) return false; if (y < mCy - dy ) return false;
	if ( z > mCz + dz ) return false; if (z < mCz - dz ) return false;
	return true;
}

//------------------------------------- Constructor, destructor -------------------------------


/*! Initializes the root to a branch with all NULL children.

  \param cx,cy,cz - the center of the Octree in space

  \param dx,dy,dz - the dimensions of the Octree along all axes

  \param maxDepth - the maximum depth of the Octree

  \param emptyValue - the value that is returned for unvisited regions
  of space.
 */

template <typename T>
Octree<T>::Octree(float cx, float cy, float cz,
	       float dx, float dy, float dz, 
	       int maxDepth, T emptyValue)

{
	mRoot = new OctreeBranch<T>();
	setCenter(cx,cy,cz);
	setSize(dx,dy,dz);
	setDepth(maxDepth);
	mEmptyValue = emptyValue;
}

//--------------------------------------- Navigation ------------------------------------------


/*! Inserts a new value at the spatial location specified by \a
    x,y,z. It is not recursive, in an attempt to be more efficient.

    If the given region of space is previously unvisited, it will
    travel down creating Branches as it goes, until reaching a depth
    of \a mMaxDepth. At that point it will create a Leaf with the
    value \a newValue.

    For now, does not aggregate Leaves together, even if all children
    of a given Branch have the same value.
 */
template <typename T>
void Octree<T>::insert(float x, float y, float z, T newValue)
{
	if (!testBounds(x,y,z)) return;

	float cx = mCx, cy = mCy, cz = mCz;
	float dx = mDx / 2.0, dy = mDy / 2.0, dz = mDz / 2.0;
	
	int depth = 0;
	unsigned char address;
	OctreeBranch<T> *currentNode = mRoot;
	OctreeNode<T> *nextNode;

	while (1) {
		dx /= 2.0;
		dy /= 2.0;
		dz /= 2.0;
		depth++;

		address = 0;
		if ( x > cx) {address += 4; cx += dx;}
		else { cx -= dx;}
		if ( y > cy) {address += 2; cy += dy;}
		else {cy -= dy;}
		if ( z > cz) {address += 1; cz += dz;}
		else {cz -= dz;}
		
		//current node is a branch by definition
		nextNode = currentNode->getChild(address);
		if (!nextNode) {
			// unexplored region of space; extend the tree
			if (depth >= mMaxDepth) {
				// we have reached max depth; create a new leaf
				nextNode = new OctreeLeaf<T>(newValue);
				currentNode->setChild(address, nextNode);
				// and we are done
				return;
			} else {
				// create a new unexplored branch
				nextNode = new OctreeBranch<T>();
				currentNode->setChild(address, nextNode);
			}
		} else if (nextNode->isLeaf()) {
			//leaf already is set to new value; we are done
			if ( ((OctreeLeaf<T>*)nextNode)->getVal()==newValue) return;

			//we have reached the max depth; set the leaf to new value then done
			if (depth >= mMaxDepth) { 
				((OctreeLeaf<T>*)nextNode)->setVal(newValue); 
				return;
			}

			//create a new branch with the all children leaves with the old value
			nextNode = new OctreeBranch<T>( ((OctreeLeaf<T>*)nextNode)->getVal() );
			currentNode->setChild(address, nextNode);
		} 

		// advance the recursion 
		currentNode = (OctreeBranch<T>*)nextNode;
	}
}

/*! Returns the value at the specified spatial coordinates. If that
    region of space is unvisited, returns \a mEmptyValue.
 */
template <typename T>
T Octree<T>::get(float x, float y, float z)
{
	if (!testBounds(x,y,z)) return mEmptyValue;

	float cx = mCx, cy = mCy, cz = mCz;
	float dx = mDx / 2.0, dy = mDy / 2.0, dz = mDz / 2.0;
	
	unsigned char address;
	OctreeBranch<T> *currentNode = mRoot;
	OctreeNode<T> *nextNode;

	while (1) {
		dx /= 2.0; dy /= 2.0; dz /= 2.0;

		address = 0;
		if ( x > cx) {address += 4; cx += dx;}
		else { cx -= dx;}
		if ( y > cy) {address += 2; cy += dy;}
		else {cy -= dy;}
		if ( z > cz) {address += 1; cz += dz;}
		else {cz -= dz;}
		
		nextNode = currentNode->getChild(address);
		if (!nextNode) {
			return mEmptyValue;
		} else if (nextNode->isLeaf()) {
			return ((OctreeLeaf<T>*)nextNode)->getVal();
		} 

		currentNode = (OctreeBranch<T>*)nextNode;
	}
}

template <typename T>
void Octree<T>::clear()
{
	delete mRoot;
	mRoot = new OctreeBranch<T>();
}

//------------------------------------------ Statistics --------------------------------------------

template <typename T>
long long unsigned int Octree<T>::getMemorySize()
{
	unsigned int leaves = mRoot->getNumLeaves();
	unsigned int branches = mRoot->getNumBranches();
	return leaves * sizeof(OctreeLeaf<T>) + branches * sizeof(OctreeBranch<T>);
}

//------------------------------------------ Serialization ------------------------------------------

/*!  This only saves the inner structure of the tree, not center,
  extents, maxdepth etc.
 */
template <typename T>
void Octree<T>::serialize(char **destinationString, unsigned int *size)
{
	int nLeaves = mRoot->getNumLeaves();
	int nBranches = mRoot->getNumBranches();

	//each branch stores a byte for each child
	//each leaf stores its value
	*size = 8 * nBranches + sizeof(mEmptyValue) * nLeaves;

	*destinationString = new char[*size];

	unsigned int address = 0;
	mRoot->serialize(*destinationString, address);
	//sanity check
	if (address != *size) {
		fprintf(stderr,"Serialization error; unexpected size\n");
	}
}

/*! It is equivalent to calling \a clear(...) first and then \a
  deserialize(...)  

  \param size - the total size of the string passed in. It is only
  used for checking correctness and avoiding memory corruption

  Return \a true if the Octree is deserialized succesfully. If it
  returns \a false, the deserialized Octree is not to be trusted.
*/

template <typename T>
bool Octree<T>::deserialize(char *sourceString, unsigned int size)
{
	unsigned int address = 0;
	mRoot->deserialize(sourceString,address, size);
	if (address != size) {
		fprintf(stderr,"Octree serialization error!\n");
		return false;
	}
	return true;
}

template <typename T>
void Octree<T>::setFromMsg(const OctreeMsg &msg)
{
	setCenter( msg.center.x, msg.center.y, msg.center.z);
	setSize(msg.size.x, msg.size.y, msg.size.z);
	setDepth(msg.max_depth);

	if (sizeof(mEmptyValue) != msg.get_empty_value_size()) {
		fprintf(stderr,"Incompatible data type in ROS Octree message!\n");
		return;
	}
	memcpy((char*)&mEmptyValue, msg.empty_value, sizeof(mEmptyValue) );

	unsigned int size = msg.get_structure_data_size();
	char *data  = new char[size];
	memcpy(data, msg.structure_data, size);
	deserialize(data,size);
	delete[] data;

	int depth = computeMaxDepth();
	if (depth > mMaxDepth) {
		fprintf(stderr,
			"Octree read from message: depth found in data is greater than maxDepth specified in header!\n");
		mMaxDepth = depth;
	}
}

template <typename T>
void Octree<T>::getAsMsg(OctreeMsg &msg)
{
	msg.center.x = mCx; msg.center.y = mCy; msg.center.z = mCz;
	msg.size.x = mDx; msg.size.y = mDy; msg.size.z = mDz;
	msg.max_depth = mMaxDepth;

	msg.set_empty_value_size(sizeof(mEmptyValue));
	memcpy(msg.empty_value, (char*)&mEmptyValue, sizeof(mEmptyValue));

	unsigned int size;
	char *data;
	serialize(&data, &size);
	msg.set_structure_data_size(size);
	memcpy(msg.structure_data, data, size);
	delete [] data;
}

template <typename T>
void Octree<T>::writeToFile(std::ostream &os)
{
	//write the admin data
	float fl[6];
	fl[0] = mCx; fl[1] = mCy; fl[2] = mCz;
	fl[3] = mDx; fl[4] = mDy; fl[5] = mDz;
	os.write((char*)fl, 6 * sizeof(float));
	os.write((char*)&mEmptyValue, sizeof(mEmptyValue));
	os.write((char*)&mMaxDepth, sizeof(int));

	//write the volume data
	unsigned int size;
	char *octreeString;
	serialize(&octreeString,&size);
	os.write(octreeString,size);
	delete [] octreeString;
}

/*
  WARNING: the structure data is assumed to span from the current
  point in the stream UNTIL EOF IS REACHED. There should be NOTHING in
  the stream following this Octree.
 */
template <typename T>
void Octree<T>::readFromFile(std::istream &is)
{
	//read the admin data
	float fl[6];
	is.read( (char*)fl, 6*sizeof(float));
	mCx = fl[0]; mCy = fl[1]; mCz = fl[2];
	mDx = fl[3]; mDy = fl[4]; mDz = fl[5];
	is.read( (char*)&mEmptyValue, sizeof(mEmptyValue) );
	is.read( (char*)&mMaxDepth, sizeof(int) );

	//read the volume data
	unsigned int current = (unsigned int)is.tellg();
	is.seekg (0, std::ios::end);
	unsigned int size = (unsigned int)is.tellg() - current;
	is.seekg (current );

	char *octreeString = new char[size];
	is.read(octreeString, size);
	if (is.fail()) {
		fprintf(stderr,"Only able to read %d instead of %d characters\n",is.gcount(), size);
	} else {
		deserialize(octreeString,size);
	}

	delete [] octreeString;
}

//--------------------------------------------- Triangulation -----------------------------------------

/*!  Returns the triangles that form the surface mesh of the NON-EMPTY
  (previously visited) cells in the Octree (regardless of the value
  they hold).

  Tries to be somewhat smart about not returning unnecesary triangles,
  but is not very good at it. A good algorithm for doing that seems to
  be an interesting problem...
 */
template <typename T>
void Octree<T>::getTriangles(std::list<Triangle> &triangles)
{
	if (!mRoot) return;
	mRoot->getTriangles( true, true, true, true, true, true, 
			     mCx, mCy, mCz, mDx/2.0, mDy/2.0, mDz/2.0, triangles);
}


} //namespace scan_utils

#endif
