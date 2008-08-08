#include "octree.h"

using namespace scan_utils;

const char Octree::NULL_CHILD = 0;
const char Octree::BRANCH = 1;
const char Octree::LEAF = 2;

/*! Initializes the root to a branch with all NULL children.

  \param cx,cy,cz - the center of the Octree in space

  \param dx,dy,dz - the dimensions of the Octree along all axes

  \param maxDepth - the maximum depth of the Octree

  \param emptyValue - the value that is returned for unvisited regions
  of space.
 */

Octree::Octree(float cx, float cy, float cz,
	       float dx, float dy, float dz, 
	       int maxDepth, int emptyValue)

{
	mRoot = new OctreeBranch();
	setCenter(cx,cy,cz);
	setSize(dx,dy,dz);
	setDepth(maxDepth);
	mEmptyValue = emptyValue;
}

void Octree::clear()
{
	delete mRoot;
	mRoot = new OctreeBranch();
}

long long unsigned int Octree::getMemorySize()
{
	unsigned int leaves = mRoot->getNumLeaves();
	unsigned int branches = mRoot->getNumBranches();
	return leaves * sizeof(OctreeLeaf) + branches * sizeof(OctreeBranch);
}

/*! Inserts a new value at the spatial location specified by \a
    x,y,z. It is not recursive, in an attempt to be more efficient.

    If the given region of space is previously unvisited, it will
    travel down creating Branches as it goes, until reaching a depth
    of \a mMaxDepth. At that point it will create a Leaf with the
    value \a newValue.

    For now, does not aggregate Leaves together, even if all children
    of a given Branch have the same value.
 */
void Octree::insert(float x, float y, float z, int newValue)
{
	if (!testBounds(x,y,z)) return;

	float cx = mCx, cy = mCy, cz = mCz;
	float dx = mDx / 2.0, dy = mDy / 2.0, dz = mDz / 2.0;
	
	int depth = 0;
	unsigned char address;
	OctreeBranch *currentNode = mRoot;
	OctreeNode *nextNode;

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
				nextNode = new OctreeLeaf(newValue);
				currentNode->setChild(address, nextNode);
				// and we are done
				return;
			} else {
				// create a new unexplored branch
				nextNode = new OctreeBranch();
				currentNode->setChild(address, nextNode);
			}
		} else if (nextNode->isLeaf()) {
			//leaf already is set to new value; we are done
			if ( ((OctreeLeaf*)nextNode)->getVal()==newValue) return;

			//we have reached the max depth; set the leaf to new value then done
			if (depth >= mMaxDepth) { 
				((OctreeLeaf*)nextNode)->setVal(newValue); 
				return;
			}

			//create a new branch with the all children leaves with the old value
			nextNode = new OctreeBranch( ((OctreeLeaf*)nextNode)->getVal() );
			currentNode->setChild(address, nextNode);
		} 

		// advance the recursion 
		currentNode = (OctreeBranch*)nextNode;
	}
}

/*! Returns the value at the specified spatial coordinates. If that
    region of space is unvisited, returns \a mEmptyValue.
 */
int Octree::get(float x, float y, float z)
{
	if (!testBounds(x,y,z)) return mEmptyValue;

	float cx = mCx, cy = mCy, cz = mCz;
	float dx = mDx / 2.0, dy = mDy / 2.0, dz = mDz / 2.0;
	
	unsigned char address;
	OctreeBranch *currentNode = mRoot;
	OctreeNode *nextNode;

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
			return ((OctreeLeaf*)nextNode)->getVal();
		} 

		currentNode = (OctreeBranch*)nextNode;
	}
}

int OctreeBranch::getNumBranches()
{
	int n=1;
	for (int i=0; i<8; i++) {
		if (mChildren[i] && !mChildren[i]->isLeaf()) {
			n += ((OctreeBranch*)mChildren[i])->getNumBranches();
		}
	}
	return n;
}

int OctreeBranch::getNumLeaves()
{
	int n=0;
	for (int i=0; i<8; i++) {
		if ( !mChildren[i] ) continue;
		if ( mChildren[i]->isLeaf()) n += 1;
		else n += ((OctreeBranch*)mChildren[i])->getNumLeaves();
	}
	return n;
}

int OctreeBranch::computeMaxDepth()
{
	int maxDepth = 0;
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) continue;
		int tmp = mChildren[i]->computeMaxDepth();
		if (tmp > maxDepth) maxDepth = tmp;
	}
	return maxDepth + 1;
}

/*!  This only saves the inner structure of the tree, not center,
  extents, maxdepth etc.
 */
void Octree::serialize(char **destinationString, unsigned int *size)
{
	int nLeaves = mRoot->getNumLeaves();
	int nBranches = mRoot->getNumBranches();

	//each branch stores a byte for each child
	//each leaf stores its value
	*size = 8 * nBranches + sizeof(int) * nLeaves;

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

bool Octree::deserialize(char *sourceString, unsigned int size)
{
	unsigned int address = 0;
	mRoot->deserialize(sourceString,address, size);
	if (address != size) {
		fprintf(stderr,"Octree serialization error!\n");
		return false;
	}
	return true;
}

void Octree::setFromMsg(const OctreeMsg &msg)
{
	setCenter( msg.center.x, msg.center.y, msg.center.z);
	setSize(msg.size.x, msg.size.y, msg.size.z);
	setDepth(msg.max_depth);
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

void Octree::getAsMsg(OctreeMsg &msg)
{
	msg.center.x = mCx; msg.center.y = mCy; msg.center.z = mCz;
	msg.size.x = mDx; msg.size.y = mDy; msg.size.z = mDz;
	msg.max_depth = mMaxDepth;
	unsigned int size;
	char *data;
	serialize(&data, &size);
	msg.set_structure_data_size(size);
	memcpy(msg.structure_data, data, size);
	delete [] data;
}

void Octree::writeToFile(std::ostream &os)
{
	//write the admin data
	float fl[6];
	fl[0] = mCx; fl[1] = mCy; fl[2] = mCz;
	fl[3] = mDx; fl[4] = mDy; fl[5] = mDz;
	os.write((char*)fl, 6 * sizeof(float));
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
void Octree::readFromFile(std::istream &is)
{
	//read the admin data
	float fl[6];
	is.read( (char*)fl, 6*sizeof(float));
	mCx = fl[0]; mCy = fl[1]; mCz = fl[2];
	mDx = fl[3]; mDy = fl[4]; mDz = fl[5];
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

void OctreeBranch::serialize(char *destinationString, unsigned int &address)
{
	for (int i=0; i<8; i++) {
		if (!mChildren[i]) {
			destinationString[address] = Octree::NULL_CHILD;
			address++;
			continue;
		}
		if (mChildren[i]->isLeaf()) {
			destinationString[address] = Octree::LEAF;
		} else {
			destinationString[address] = Octree::BRANCH;
		}
		address++;
		mChildren[i]->serialize(destinationString, address);
	}
}

void OctreeBranch::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	for (int i=0; i<8; i++) {
		if (address >= size) return;
		if (sourceString[address] == Octree::NULL_CHILD) {
			setChild(i,NULL);
			address++;
			continue;
		}
		if (sourceString[address] == Octree::LEAF) {
			setChild(i, new OctreeLeaf() );

		} else if (sourceString[address] == Octree::BRANCH) {
			setChild(i, new OctreeBranch() );
		} else {
			//error
			address = size;
			return;
		}
		address++;
		mChildren[i]->deserialize(sourceString, address, size);
	}

}

void OctreeLeaf::serialize(char *destinationString, unsigned int &address)
{
	memcpy(&destinationString[address], &mValue, sizeof(int));
	address += sizeof(int);
}

void OctreeLeaf::deserialize(char *sourceString, unsigned int &address, unsigned int size)
{
	if (address + sizeof(int) > size) {
		address = size;
		return;
	}
	memcpy(&mValue, &sourceString[address], sizeof(int));
	address += sizeof(int);
}

/*!  Returns the triangles that form the surface mesh of the NON-EMPTY
  (previously visited) cells in the Octree (regardless of the value
  they hold).

  Tries to be somewhat smart about not returning unnecesary triangles,
  but is not very good at it. A good algorithm for doing that seems to
  be an interesting problem...
 */
void Octree::getTriangles(std::list<Triangle> &triangles)
{
	if (!mRoot) return;
	mRoot->getTriangles( true, true, true, true, true, true, 
			     mCx, mCy, mCz, mDx/2.0, mDy/2.0, mDz/2.0, triangles);
}

void OctreeBranch::getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
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
			if ( mChildren[i+4] && mChildren[i+4]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i+4])->getVal() == 1) nextpx = false;
			else nextpx = true;			
		} else {
			nextx = cx+dx;
			nextpx = px;
			if ( mChildren[i-4] && mChildren[i-4]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i-4])->getVal() == 1) nextnx = false;
			else nextnx = true;			
		}

		if ( (i%4)/2 == 0 ) {
			nexty = cy-dy;
			nextny = ny;
			if ( mChildren[i+2] && mChildren[i+2]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i+2])->getVal() == 1) nextpy = false;
			else nextpy = true;			
		} else {
			nexty = cy+dy;
			nextpy = py;
			if ( mChildren[i-2] && mChildren[i-2]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i-2])->getVal() == 1) nextny = false;
			else nextny = true;			
		}

		if ( (i%4)%2 == 0 ) {
			nextz = cz-dz;
			nextnz = nz;
			if ( mChildren[i+1] && mChildren[i+1]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i+1])->getVal() == 1) nextpz = false;
			else nextpz = true;			
		} else {
			nextz = cz+dz;
			nextpz = nz;
			if ( mChildren[i-1] && mChildren[i-1]->isLeaf() 
			     && ((OctreeLeaf*)mChildren[i-1])->getVal() == 1) nextnz = false;
			else nextnz = true;			
		}

		mChildren[i]->getTriangles( nextpx, nextnx, nextpy, nextny, nextpz, nextnz, 
					    nextx, nexty, nextz, dx, dy, dz, triangles);
	}
}

void OctreeLeaf::getTriangles(bool px, bool nx, bool py, bool ny, bool pz, bool nz,
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
