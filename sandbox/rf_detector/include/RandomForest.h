#pragma once

#include <vector>
#include "librf/random_forest.h"
// #include "FeatureExtractor.h"
using namespace std;


class CRandomForest
{
public:
private:
	int dim;
	bool m_bLoad;
	int num_trees;
	int K;
	int min_size;
	float min_gain;
	int max_depth;
	int * tree_size;
	librf::RandomForest librf;
	bool m_bRandomSplit;
	int m_iSplitNum;
	float m_fSampleRatio;
	bool m_bTreeSplit;

public:
	CRandomForest():dim(-1),
			        m_bLoad(false),
					num_trees(-1),
					min_size(10),
					min_gain(0.0f),
					max_depth(20),
					K(-1),
					m_bRandomSplit(false),
					m_fSampleRatio(0.5),
					m_iSplitNum(-1){};
	int RFTrain(IN vector< vector<float> > & vecUniverse,
				IN vector<float> &vecUniverseWeight,
				OUT vector<int> & vecFgbgLabel,
				OUT vector<int> & vecViewpartLabel,
				IN string modelfile);
	float RFPredict(IN float *input, OUT int * output);
	// float RFPredict(IN float *input, OUT int * output, IN CFeatureExtractor & fe);
	void RFPredict(IN vector< vector<float> > &vecFeature,
				   IN int label,
				   OUT vector<float> &vecPredict);

	void RFPredict(IN vector< vector<float> > &vecFeature,
		OUT vector< vector<int> > &vecPredict);

	int RFLoad(string modelfile);
	int RFGetTreeSize(int n);
	int RFGetTreesNum();
	void RFDestroy();
	void SetK(IN int iK) { K = iK; }

	void SetMinSize(IN int iMinSize) { min_size = iMinSize; }

	void SetMinGain(IN float fMinGain) { min_gain = fMinGain; }

	void SetNumTrees(IN int itrees)	{	num_trees = itrees;	}
	int GetNumTrees() { return num_trees; }

	void SetSplitNum(IN int iSplitNum) { m_iSplitNum = iSplitNum; }

	void SetMaxDepth(IN int idepth)	{	max_depth = idepth;	}
	void SetDim(IN int idim) { dim = idim; }
	void SetLoadStatus(IN bool bLoad) { m_bLoad = bLoad; }
	bool GetLoadStatus() { return m_bLoad; }
	void SetRandomSplit(IN bool bSplit) { m_bRandomSplit = bSplit; }
	void SetSampleRatio(IN float fSampleRatio) { m_fSampleRatio = fSampleRatio; }
	void SetTreeSplit(IN bool bTreeSplit) { m_bTreeSplit = bTreeSplit; }

    vector<int>* get_vecpatchIds( int treeId, int terminialLocalId);
};
