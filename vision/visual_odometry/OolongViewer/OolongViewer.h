/*
 * OolongViewer.h
 *
 *  Created on: Aug 19, 2008
 *      Author: jdchen
 */

#ifndef OOLONGVIEWER_H_
#define OOLONGVIEWER_H_

#include <opencv/cxtypes.h>

#include <string>
#include <vector>
#include <vtkActor.h>
using namespace std;

class OolongViewer {
public:
	OolongViewer();
	virtual ~OolongViewer();

	void view(const char*dirname = NULL, int id=0);
	void view(const int numDirs, const char *dirnames[] = NULL);
	CvMat* loadTrajectory(const char *filename = NULL);
	CvMat* loadRotations(const char *filename = NULL);
	CvMat* loadPointCloud(const string& filename);
	CvMat* loadFramePoses(const string& filename);
	void viewTrajectory(CvMat& trajectory);

protected:
	vector<vtkActor*> path_actors_;
	CvPoint3D64f end_point_;
};

#endif /* OOLONGVIEWER_H_ */
