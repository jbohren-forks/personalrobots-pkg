#ifndef _octreenodes_h_
#define _octreenodes_h_

#include <stdlib.h>
#include <list>
#include <dataTypes.h>

namespace scan_utils{

//! Constants that need to be chars to save space
namespace OctreeChildType {
const char NULL_CHILD = 0;
const char BRANCH = 1;
const char LEAF = 2;
}

/*!  A generic Octree node, could either be a leaf or a branch. Could
  be defined privately inside the Octree class, but in the future we
  might implement faster accessors that return the node that they
  accessed, so that subsequent calls can use this information.
*/
template <typename T>
class OctreeNode {
 private:
 public:
	OctreeNode(){}
	virtual bool isLeaf() = 0;
	virtual void getTriangles(bool, bool, bool, bool, bool, bool,
				  float, float, float,
				  float, float, float,
				  std::list<Triangle>&){}
	virtual void serialize(char*, unsigned int&){}
	virtual void deserialize(char*, unsigned int&, unsigned int){}
	virtual int computeMaxDepth(){return 0;}

};


/*! An Octree branch. Always contains exactly 8 children pointers. A
    NULL child pointer means that the respective child points to an
    unexplored region of space and thus is equivalent to having a
    child with the mEmptyValue of the Octree set.
 */
template <typename T>
class OctreeBranch : public OctreeNode<T> {
 private:
	OctreeNode<T> **mChildren;
 public:
	bool isLeaf(){return false;}

	//! Initializes a branch with all NULL (unexplored) children
	inline OctreeBranch();
	//! Initializes a branch with all children set to the value \a val 
	inline OctreeBranch(T val);
	//! Destructor will delete all children first. Thus, delete an Octree top-down by just deleting its root.
	inline ~OctreeBranch();

	//! Return the child at address \a adress, between 0 and 7
	OctreeNode<T>* getChild(unsigned char address) { return mChildren[address]; }
	//! Sets the child at address \a adress to point at \a child. 
	/*! If a child was already present at that address it is deleted.*/
	inline void setChild(unsigned char address, OctreeNode<T> *child);
	//! Recursively returns the total number of branches below this one (including this one)
	int getNumBranches();
	//! Recursively returns the number of leaves between this one
	int getNumLeaves();
	//! Recursively return the triangles that form the surface of non-empty cells below this branch.
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
template <typename T>
class OctreeLeaf : public OctreeNode<T> {
 private:
	T mValue;
 public:
        OctreeLeaf(T val) {setVal(val);}
	OctreeLeaf(){}
	~OctreeLeaf(){}
	bool isLeaf(){return true;}

	T getVal(){return mValue;}
	void setVal(T val){mValue = val;}
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

//------------------------------------ Constructors and destructors -------------------------

template <typename T>
OctreeBranch<T>::OctreeBranch()
{
	mChildren = new OctreeNode<T>*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = NULL;
	}
}	

template <typename T>
OctreeBranch<T>::OctreeBranch(T val)
{
	mChildren = new OctreeNode<T>*[8];
	for (int i=0; i<8; i++) {
		mChildren[i] = new OctreeLeaf<T>(val);
	}
}	

template <typename T>
OctreeBranch<T>::~OctreeBranch()
{
	for(int i=0; i<8; i++) {
		if (mChildren[i]) delete mChildren[i];
	}
	delete mChildren;
}

//------------------------------------- Navigation ------------------------------------------
 
template <typename T>
void OctreeBranch<T>::setChild(unsigned char address, OctreeNode<T> *child) 
{
	if (mChildren[address]) delete mChildren[address];
	mChildren[address] = child; 
}

//-------------------------------------- Statistics -----------------------------------------

template <typename T>
int OctreeBranch<T>::getNumBranches()
{
	int n=1;
	for (int i=0; i<8; i++) {
		if (mChildren[i] && !mChildren[i]->isLeaf()) {
			n += ((OctreeBranch*)mChildren[i])->getNumBranches();
		}
	}
	return n;
}

template <typename T>
int OctreeBranch<T>::getNumLeaves()
{
	int n=0;
	for (int i=0; i<8; i++) {
		if ( !mChildren[i] ) continue;
		if ( mChildren[i]->isLeaf()) n += 1;
		else n += ((OctreeBranch*)mChildren[i])->getNumLeaves();
	}
	return n;
}

template <typename T>
int OctreeBranch<T>::computeMaxDepth()
{
	int maxDepth = 0;
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) continue;
		int tmp = mChildren[i]->computeMaxDepth();
		if (tmp > maxDepth) maxDepth = tmp;
	}
	return maxDepth + 1;
}

//----------------------------------------- Serialization -----------------------------------

template <typename T>
void OctreeBranch<T>::serialize(char *destinationString, unsigned int &address)
{
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) {
			destinationString[address] = OctreeChildType::NULL_CHILD;
			address++;
			continue;
		}
		if (mChildren[i]->isLeaf()) {
			destinationString[address] = OctreeChildType::LEAF;
		} else {
			destinationString[address] = OctreeChildType::BRANCH;
		}
		address++;
		mChildren[i]->serialize(destinationString, address);
	}
}

template <typename T>
void OctreeBranch<T>::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	for (int i=0; i<8; i++) {
		if (address >= size) return;
		if (sourceString[address] == OctreeChildType::NULL_CHILD) {
			setChild(i,NULL);
			address++;
			continue;
		}
		if (sourceString[address] == OctreeChildType::LEAF) {
			setChild(i, new OctreeLeaf<T>() );

		} else if (sourceString[address] == OctreeChildType::BRANCH) {
			setChild(i, new OctreeBranch<T>() );
		} else {
			//error
			address = size;
			return;
		}
		address++;
		mChildren[i]->deserialize(sourceString, address, size);
	}

}

template <typename T>
void OctreeLeaf<T>::serialize(char *destinationString, unsigned int &address)
{
	memcpy(&destinationString[address], &mValue, sizeof(mValue));
	address += sizeof(mValue);
}

template <typename T>
void OctreeLeaf<T>::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	if (address + sizeof(mValue) > size) {
		address = size;
		return;
	}
	memcpy(&mValue, &sourceString[address], sizeof(mValue));
	address += sizeof(mValue);
}

//--------------------------------------------- Triangulation --------------------------------

template <typename T>
void OctreeBranch<T>::getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
				float cx, float cy, float cz,
				float dx, float dy, float dz,
				std::list<Triangle> &triangles)
{
	dx/=2.0; dy/=2.0; dz/=2.0;

	float nextx, nexty, nextz;
	bool nextpx, nextnx, nextpy, nextny, nextpz, nextnz;
	
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) continue;

		if (i/4 == 0) {
			nextx = cx-dx;
			nextnx = nx;
			if ( mChildren[i+4] && mChildren[i+4]->isLeaf() ) nextpx = false;
			else nextpx = true;			
		} else {
			nextx = cx+dx;
			nextpx = px;
			if ( mChildren[i-4] && mChildren[i-4]->isLeaf() ) nextnx = false;
			else nextnx = true;			
		}

		if ( (i%4)/2 == 0 ) {
			nexty = cy-dy;
			nextny = ny;
			if ( mChildren[i+2] && mChildren[i+2]->isLeaf() ) nextpy = false;
			else nextpy = true;			
		} else {
			nexty = cy+dy;
			nextpy = py;
			if ( mChildren[i-2] && mChildren[i-2]->isLeaf() ) nextny = false;
			else nextny = true;			
		}

		if ( (i%4)%2 == 0 ) {
			nextz = cz-dz;
			nextnz = nz;
			if ( mChildren[i+1] && mChildren[i+1]->isLeaf() ) nextpz = false;
			else nextpz = true;			
		} else {
			nextz = cz+dz;
			nextpz = pz;
			if ( mChildren[i-1] && mChildren[i-1]->isLeaf() ) nextnz = false;
			else nextnz = true;			
		}

		mChildren[i]->getTriangles( nextpx, nextnx, nextpy, nextny, nextpz, nextnz, 
					    nextx, nexty, nextz, dx, dy, dz, triangles);
	}
}

template <typename T>
void OctreeLeaf<T>::getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
			      float cx, float cy, float cz,
			      float dx, float dy, float dz,
			      std::list<Triangle> &triangles)
{
	if (px) {
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz,
					       cx+dx, cy-dy, cz-dz,
					       cx+dx, cy+dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz,
					       cx+dx, cy-dy, cz+dz,
					       cx+dx, cy-dy, cz-dz ) );
	}
	if (nx) {
		triangles.push_back( Triangle( cx-dx, cy+dy, cz+dz,
					       cx-dx, cy+dy, cz-dz,
					       cx-dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx-dx, cy+dy, cz+dz,
					       cx-dx, cy-dy, cz-dz,
					       cx-dx, cy-dy, cz+dz ) );
	}

	if (py) {
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz, 
					       cx+dx, cy+dy, cz-dz,
					       cx-dx, cy+dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy+dy, cz+dz, 
					       cx-dx, cy+dy, cz-dz,
					       cx-dx, cy+dy, cz+dz ) );
	}
	if (ny) {
		triangles.push_back( Triangle( cx+dx, cy-dy, cz+dz, 
					       cx-dx, cy-dy, cz-dz,
					       cx+dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx+dx, cy-dy, cz+dz, 
					       cx-dx, cy-dy, cz+dz,
					       cx-dx, cy-dy, cz-dz ) );
	}

	if(pz) {
		triangles.push_back( Triangle( cx-dx, cy-dy, cz+dz,
					       cx+dx, cy-dy, cz+dz,
					       cx+dx, cy+dy, cz+dz ) );
		triangles.push_back( Triangle( cx-dx, cy-dy, cz+dz,
					       cx+dx, cy+dy, cz+dz,
					       cx-dx, cy+dy, cz+dz ) );
	}
	if(nz) {
		triangles.push_back( Triangle( cx-dx, cy-dy, cz-dz,
					       cx+dx, cy+dy, cz-dz,
					       cx+dx, cy-dy, cz-dz ) );
		triangles.push_back( Triangle( cx-dx, cy-dy, cz-dz,
					       cx-dx, cy+dy, cz-dz,
					       cx+dx, cy+dy, cz-dz ) );
	}
}

} //namespace scan_utils


#endif
