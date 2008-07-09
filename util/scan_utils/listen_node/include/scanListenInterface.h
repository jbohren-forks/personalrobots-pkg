#ifndef _rosinterface_h_
#define _rosinterface_h_

#include <vector>

class RosScanNode;
struct cloud_point;

class RosInterface
{
 private:
	RosScanNode *mScanNode;
 public:
	RosInterface();
	~RosInterface();

	std::vector<cloud_point>* getFullCloud();
	std::vector<cloud_point>* getCurrentCloud();

	void writeCloudToFile(std::vector<cloud_point>* cloud, std::iostream &output);
	std::vector<cloud_point>* readCloudFromFile(std::iostream &input);
};

#endif
