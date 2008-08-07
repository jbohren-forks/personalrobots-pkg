#include "octree.h"

using namespace scan_utils;

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
	mCx = cx; mCy = cy; mCz = cz;
	mDx = dx; mDy = dy; mDz = dz;
	mMaxDepth = maxDepth;
	if (mMaxDepth==0) mMaxDepth = 1;
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
