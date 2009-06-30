// RandomForest.cpp : Defines the entry point for the console application.
//
#include "librf/librf.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include "RandomForest.h"


#define round(x) (x<0?ceil((x)-0.5):floor((x)+0.5))

using namespace std;
using namespace librf;

int CRandomForest::RFGetTreesNum()
{
	return librf.get_trees_size();
}
void CRandomForest::RFDestroy()
{
	if (m_bLoad)
	{
		librf.destroy();
	}
}
int CRandomForest::RFTrain(IN vector< vector<float> > & vecUniverse,
						   IN vector<float> &vecUniverseWeight,
						   OUT vector<int> & vecFgbgLabel,
						   OUT vector<int> & vecViewpartLabel,
						   IN string modelfile)
{
	InstanceSet* set = NULL;

	int set_size;

	set = InstanceSet::load_data_and_labels(vecUniverse, vecFgbgLabel, vecViewpartLabel);

	if (set->size() == 0)
	{
		cout << "Error in load data " << endl;
		return -1;
	}
	set_size = set->size();

	//general
	if (K == -1) {
		K = int(sqrt(double(set->num_attributes())));
	}

	//decision tree
	if (K == -2){
		K = set->num_attributes();
	}

	librf.build(*set, num_trees, K, min_size, min_gain, max_depth, m_fSampleRatio, m_bRandomSplit, m_iSplitNum, m_bTreeSplit, vecUniverseWeight);


	ofstream out(modelfile.c_str());
	librf.write(out);
	out.close();
	modelfile = modelfile + ".verbose";
	out.open(modelfile.c_str());
	librf.writeverbose(out);
	out.close();
	set->save_data_and_labels(vecUniverse, vecFgbgLabel, vecViewpartLabel);

	delete set;

	return 0;

}
int CRandomForest::RFGetTreeSize(int n)
{
	return librf.get_tree_size(n);
}
int CRandomForest::RFLoad(string modelfile)
{
	printf("Loading random forest...\n");
	ifstream in(modelfile.c_str());
	if (in.fail())
	{
		cout << "Error in load data " << modelfile << endl;
		return -1;
	}
	librf.read(in);
	num_trees = librf.get_trees_size();
	cout << "Load tree size: " << num_trees << endl;
	if (num_trees > 0)
	{
		SetLoadStatus(true);
		printf("Done.\n");
		return 0;
	}
	return -1;
}

void CRandomForest::RFPredict(IN vector< vector<float> > &vecFeature,
							  IN int label,
							  OUT vector<float> &vecPredict)
{
	if (!m_bLoad)
	{
		printf("model is not loaded.\n");
		return;
	}
	vecPredict.resize(vecFeature[0].size());
	float *fFeature = new float [vecFeature.size()];
	int *fLeaves = new int [librf.get_trees_size()];
	for (int i = 0; i < vecFeature[0].size(); i++)
	{
		//predict is the probability of being 1
		for (int j = 0; j < vecFeature.size(); j++)
		{
			fFeature[j] = vecFeature[j][i];
		}
		vecPredict[i] = librf.predict_prob_singlefeature(fFeature, label, fLeaves);
	}
	delete fFeature;
	delete fLeaves;
}

void CRandomForest::RFPredict(IN vector< vector<float> > &vecFeature,
							  OUT vector< vector<int> > &vecPredict )
{
	if (!m_bLoad)
	{
		printf("model is not loaded.\n");
		return;
	}
	vecPredict.resize(vecFeature[0].size());
	float *fFeature = new float [vecFeature.size()];
	for (int i = 0; i < vecFeature[0].size(); i++)
	{
		vecPredict[i].resize(librf.get_trees_size());
		//predict is the probability of being 1
		for (int j = 0; j < vecFeature.size(); j++)
		{
			fFeature[j] = vecFeature[j][i];
		}
		librf.predict_prob_singlefeature(fFeature, 0, &vecPredict[i][0]);
	}
	delete fFeature;
}

float CRandomForest::RFPredict(float *input, int * output)
{
	if (!m_bLoad)
	{
		printf("model is not loaded.\n");
		return -1.0f;
	}

	return librf.predict_prob_singlefeature(input, 1, output);
}

vector<int>* CRandomForest::get_vecpatchIds( int treeId, int terminialLocalId)
{
//    vector<int>* vecpatchIdptr = librf.get_vecpatchIds( treeId,  terminialLocalId);
//    cout << vecpatchIdptr->at(0) << endl;
    return librf.get_vecpatchIds( treeId,  terminialLocalId);
}
//float CRandomForest::RFPredict(float *input, int * output, CFeatureExtractor & fe)
//{
//	if (!m_bLoad)
//	{
//		printf("model is not loaded.\n");
//		return -1.0f;
//	}
//	return librf.predict_prob_singlefeature(input, 1, output, fe);
//}
