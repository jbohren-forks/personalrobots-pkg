/**
* @file
* @brief Tree implementation
* TODO: fix nodes to be contiguous
* use ncur idea
* if current node is split
* last_node +1 (left)
* last_node +2 (right)
* ncur+=2
*/
//#include "stdafx.h"
#include <float.h>
#include "tree.h"
#include "instance_set.h"
#include "discrete_dist.h"
#include "weights.h"
#include "utils.h"
#include "limits.h"

// #include "FeatureExtractor.h"
// binary tree implicit in array
// rows in sorted_inum matrix are arranged in a similar style
extern int NView;
extern float fgbgSampleRatio;
namespace librf {

	const int Tree::kLeft = 0;
	const int Tree::kRight = 1;

	Tree::Tree(istream& in):
	// if we load the tree from disk, there is no training data set
	set_(InstanceSet()),
		// also there is no list of weights
		weight_list_(NULL),
		sorted_inum_(NULL),
		temp(NULL),
		move_left(NULL)
	{
		read(in);
	}
	/**
	* Constructor
	* @param set training set
	* @param weights weights --- presumably from bagging process
	* @param K number of vars to try per split
	* @param min_size minimum number of instances in a node
	* @param min_gain minimum information gain for making a split
	* @param seed random seed
	*/
	Tree::Tree(const InstanceSet& set,
		weight_list* weights,
		int iCurNumInstance,
		int K,
		int min_size,
		float min_gain,
		int max_depth,
		bool brandomsplit,
		int iSplitNum,
		bool bTreeSplit) :
	set_(set),
		weight_list_(weights),
		K_(K),
		min_size_(min_size),
		min_gain_(min_gain),
		max_depth_(max_depth),
		num_attributes_(set.num_attributes()),
		num_instances_(iCurNumInstance),
		brandomsplit_(brandomsplit),
		iSplitNum_(iSplitNum),
		bTreeSplit_(bTreeSplit)
	{
		min_gain_split_ = 25.0f;
		split_nodes_ = 0;
		terminal_nodes_ = 0;
	}
	/*
	* Copy the sorted indices from the training set
	* into our special matrix (sorted_inum)
	* Instance number may smaller than the set size due to sample
	*/
	void Tree::copy_instances() {
		sorted_inum_ = new int*[num_attributes_];
		for (int i = 0; i <num_attributes_; ++i) {
			const vector<int>& sorted = set_.get_sorted_indices(i);
			sorted_inum_[i] = new int[num_instances_];
			int iCurrent = 0;
			for (int j = 0; j < set_.size(); ++j)
			{
				if ((*weight_list_)[sorted[j]] > 0)
				{
					sorted_inum_[i][iCurrent++] = sorted[j];
				}
			}
		}
		temp = new int[num_instances_];
		move_left = new int[set_.size()];
	}

	/**
	* Do the work of growing the tree
	* - Copy the data into special matrix
	* - Build the tree
	* - Delete special matrix
	*/
	void Tree::grow() {
		copy_instances();
		printf("real start!");
		build_tree();
		// delete sorted_inum
		if (sorted_inum_ != NULL) {
			for (int i = 0; i <num_attributes_; ++i) {
				delete [] sorted_inum_[i];
			}
			delete [] sorted_inum_;
		}
		delete [] temp;
		delete [] move_left;
	}
	Tree::~Tree() {
		delete weight_list_;
	}
	/** Save a tree to disk
	* Important things to record:
	*    - Nodes (all active nodes need to be written)
	*    - WeightList ? - this seems irrelevant without the instance set
	*    - Statistics?
	*/
	void Tree::write(ostream& o) const {
		o << "Tree: " << vecnodes_.size() << " " << terminal_nodes_ << endl;
		// Loop through active nodes
		for (int i = 0; i < vecnodes_.size(); ++i) {
			// Write the node number
			o << i << " ";
			vecnodes_.at(i).write(o);
		}
	}

	void Tree::writeverbose( ostream& o ) const
	{
		o << "Tree: " << vecnodes_.size() << " " << terminal_nodes_ << endl;
		// Loop through active nodes
		for (int i = 0; i < vecnodes_.size(); ++i) {
			// Write the node number
			o << i << " ";
			vecnodes_.at(i).writeverbose(o);
		}
	}


	/**
	* Read the tree from disk
	*/
	void Tree::read(istream& in) {
		string spacer;
		int num_nodes;
		bool TerminalFlag;
		in >> spacer >> num_nodes >> terminal_nodes_;
		vecnodes_.resize( num_nodes);
		for (int i = 0; i < num_nodes; ++i) {
			int cur_node;
			in >> cur_node;
			TerminalFlag = vecnodes_.at(cur_node).read(in);
			if (TerminalFlag)
                TerminalNodeGlobalNodelIds_.push_back(cur_node);
		}
	}

	void Tree::build_tree() {
		int built_nodes = 0;
		// set up ROOT NODE (contains all instances)
		add_node(0, num_instances_, 0, BUILD_ME, 0);
		do {
			if (!bTreeSplit_)
			{
				build_node(built_nodes);
			}
			//not used anymore
			//else
			//{
			//	build_node_test(built_nodes);
			//}
			built_nodes++;
		} while (built_nodes < vecnodes_.size());
	}
	int Tree::get_terminal_nodes(){
		return (terminal_nodes_);
	}

	void Tree::mark_terminal(tree_node* n) {
		//record the para
		n->status = TERMINAL;
		n->leaf_no = terminal_nodes_;
		int nend = n->start + n->size;
		terminal_nodes_++;
		for (int i = n->start; i < nend; ++i)
		{
			int instance = sorted_inum_[0][i];
			n->vecpatchid.push_back(instance);
		}
	}

	void Tree::mark_split( tree_node* n, int split_attr, float split_point )
	{
		n->status = SPLIT;
		n->attr = split_attr;
		n->split_point = split_point;
		n->left = vecnodes_.size(); // last_node + 1 (due to zero indexing)
		n->right = vecnodes_.size() + 1; // last_node +2
		split_nodes_++;
		vars_used_.insert(split_attr);
	}

	void Tree::add_node( int start, int size, int depth, NodeStatusType nodestatus, float gain_parent )
	{
		tree_node n1;
		n1.status = nodestatus;
		n1.start = start;
		n1.size = size;
		n1.depth = depth;
		n1.gain = gain_parent;
		vecnodes_.push_back(n1);
	}

	void Tree::build_node(int node_num) {
		printf("start to split node!");
		assert(node_num < vecnodes_.size());
		tree_node* n = &vecnodes_.at(node_num);
		// Calculate starting entropy
		int nstart = n->start;
		int nend = n->start + n->size;

		// for fgbg
		DiscreteDist d_fgbg(2), d_viewpart(NView);
		for (int i = n->start; i < nend; ++i) {
			int instance = sorted_inum_[0][i];
			d_fgbg.add(set_.fgbglabel(instance), (*weight_list_)[instance]);
		}
		n->fgbgentropy = d_fgbg.entropy_over_classes();
		n->fgbglabel = d_fgbg.mode();
		d_fgbg.copydist(n->fgbgdist);

		for (int i = n->start; i < nend; ++i) {
			int instance = sorted_inum_[0][i];
			// if (set_.viewpartlabel(instance) != 0)
				d_viewpart.add(set_.viewpartlabel(instance), (*weight_list_)[instance]);
		}
		n->viewentropy = d_viewpart.entropy_over_classes();
		n->viewpartlabel = d_viewpart.mode();
		d_viewpart.copydist(n->viewdist);

		//d.copydist(n->dist);
		int SPLIT_METHOD = 0;

		/*if (n->fgbgentropy < 1e-2)
			SPLIT_METHOD = 1;*/
		if (rand() < RAND_MAX * fgbgSampleRatio)
			SPLIT_METHOD = 0;
		else
			SPLIT_METHOD = 1;

		// for fgbg
		if (SPLIT_METHOD == 0)
		{
			if (n->size <= min_size_ || n->fgbgentropy <= min_gain_ || n->depth >= (max_depth_ -1))
			{
				mark_terminal(n);
				return;
			}
		}
		else if (SPLIT_METHOD == 1)
		{
			if (n->size <= min_size_ || n->viewentropy <= min_gain_ || n->depth >= (max_depth_ -1))
			{
				if (n->size > 200)
					printf("huge node n->size=%d fgbglabel=%d viewpartlabel=%d ", n->size, n->fgbglabel, n->viewpartlabel);
				if (n->size <= min_size_)
					printf(" stop 4 min_size_=%d\n", min_size_);
				else if (n->viewentropy <= min_gain_)
					printf(" stop 5 %f\n", n->viewentropy);
				else if (n->depth >= (max_depth_ -1))
					printf(" stop 6\n");
				printf("\n");

				mark_terminal(n);
				return;
			}
		}

		int split_attr, split_idx;
		float split_point, split_gain;
		vector<int> attrs;

		/*if (SPLIT_METHOD == 0)*/
			random_sample(num_attributes_, K_, &attrs);
		/*else if (SPLIT_METHOD == 1)
			random_sample(num_attributes_, K_ * K_, &attrs);*/

		find_best_split(n, attrs, &split_attr, &split_idx, &split_point, &split_gain, SPLIT_METHOD);
		int left_size = split_idx - n->start + 1;
		int right_size = n->size - left_size;
		if ((left_size > min_size_ && right_size > min_size_) && split_gain > min_gain_)
		{
			mark_split(n, split_attr, split_point);
			move_data(n, split_attr, split_idx);
			int depth = n->depth;
			int start = n->start;
			if (SPLIT_METHOD == 0)
				;
			else if (SPLIT_METHOD == 1)
				split_gain /= log((float)NView);
			add_node(start, left_size, depth + 1, BUILD_ME, split_gain);
			add_node(split_idx + 1, right_size, depth + 1, BUILD_ME, split_gain);
		}
		else
		{
			mark_terminal(n);
			if (n->size > 200)
				printf("huge node fgbglabel=%d viewpartlabel=%d ", n->fgbglabel, n->viewpartlabel);
			if (split_idx < 0)
				printf(" stop 1 split_idx=%d\n", split_idx);
			if (split_gain < min_gain_)
				printf(" stop 2 split_gain=%f\n", split_gain);
			if (left_size < min_size_ && right_size < min_size_)
				printf(" stop 3\n");
			printf("\n");
		}

//		printf("split one node!");
	}

	void Tree::move_data(tree_node* n, int split_attr, int split_idx) {
		// PRE-CONDITION
		// the same number of distinct case numbers are found in
		// sorted_inum_[m][nstart-nend] for all m

		// Step 1:
		// Create an indicator bit set for moving left
		// ex. move_left[instance number]
		// vector<uchar> move_left(set_.size(), 0);
		int nstart = n->start;
		int nend = nstart + n->size;
		for (int i =0; i < set_.size(); ++i) {
			move_left[i] = 0;
		}
		// int split_stride = split_attr * stride_;
		for (int i = nstart; i <=split_idx; ++i) {
			//int instance_num = sorted_inum_[split_stride + i];
			int instance_num = sorted_inum_[split_attr][i];
			move_left[instance_num] = 1;
		}

		// Step 2:
		// For every attribute
		//    fill a temporary vector -- move left | move right
		//    write this back to the sorted_inum_
		// vector<int> temp(n->size);
		for (int attr = 0; attr < num_attributes_; ++attr) {
			int left = n->start;
			int right = split_idx + 1;
			// int attr_stride = attr * stride_;
			// Move instance numbers Left and right
			for (int i = nstart; i < nend; ++i) {
				// int instance_num = sorted_inum_[attr_stride + i];
				int instance_num = sorted_inum_[attr][i];
				//cout << i << " ";
				//cout << instance_num << " ";
				//cout << int(move_left[instance_num]) << endl;
				if (move_left[instance_num]) {
					assert(left < num_instances_);
					temp[left++] = instance_num;
				} else {
					assert(right < num_instances_);
					temp[right++] = instance_num;
				}
			}
			// Write temp back to sorted_inum
			for (int i = nstart; i < nend; ++i) {
				//sorted_inum_[attr_stride + i] = temp[i];
				sorted_inum_[attr][i] = temp[i];
			}
		}
		// POST-CONDITION
		// sorted_instances_[m][nstart-split] are consistent
		// sorted_instances_[m][split-nend] are consistent
	}
	int d = 0;
	int inst_no_debug, attr_debug;
	void Tree::find_best_split( tree_node* n, const vector<int>& attrs, int* split_attr, int* split_idx, float* split_point, float* split_gain, int SPLIT_METHOD )
	{
		float best_gain = -FLT_MAX;
		int best_attr = -1;
		int best_split_idx = -1;
		float best_split_point = -FLT_MAX;

		for (int i = 0; i < attrs.size(); ++i) {
			int attr =attrs[i];

			// cout << "investigating attr #" << attr <<endl;
			int curr_split_idx = -999;
			float curr_split_point = -999;
			float curr_gain = -FLT_MAX;
			if (brandomsplit_)
			{
				get_random_split_for_attr(n, attr, n->fgbgentropy, &curr_split_idx,
					&curr_split_point, &curr_gain);
			}
			else
			{
				if (SPLIT_METHOD == 0)
					find_best_split_for_attr(n, attr, n->fgbgentropy, &curr_split_idx,
						&curr_split_point, &curr_gain, SPLIT_METHOD);
				else if (SPLIT_METHOD == 1)
					find_best_split_for_attr(n, attr, n->viewentropy, &curr_split_idx,
						&curr_split_point, &curr_gain, SPLIT_METHOD);
				// cout << attr << ":" << curr_split_point << "->" << curr_gain <<endl;
			}
			if (curr_gain > best_gain) {
				best_gain = curr_gain;
				best_split_idx = curr_split_idx;
				best_split_point = curr_split_point;
				best_attr = attr;
				assert(best_split_idx >=0);
				assert(best_split_idx < num_instances_);
			}
		}
		// get the split point
		*split_point = best_split_point;
		*split_attr = best_attr;
		*split_idx = best_split_idx;
		*split_gain = best_gain;
	}

	void Tree::find_best_split_for_attr( tree_node* n, int attr, float prior_entropy, int* split_idx, float *split_point, float* best_gain, int SPLIT_METHOD )
	{
		int nstart = n->start;
		int nend = n->start + n->size;

		vector<DiscreteDist> split_fgbgdist;
		split_fgbgdist.resize(2, DiscreteDist(2));
		vector<DiscreteDist> split_viewpartdist;
		split_viewpartdist.resize(2, DiscreteDist(NView));
		// Move all the instances into the right split at first

		for (int i = nstart; i < nend; ++i) {
			//int inst_no = sorted_inum_[attr_stride + i];
			int inst_no = sorted_inum_[attr][i];

			if (SPLIT_METHOD == 0)
				split_fgbgdist[kRight].add(set_.fgbglabel(inst_no),
					(*weight_list_)[inst_no]);
			else if (SPLIT_METHOD == 1)
			{
				int label = set_.viewpartlabel(inst_no);
				//if (label != 0)
					split_viewpartdist[kRight].add(label,
					(*weight_list_)[inst_no]);
			}
		}
		*best_gain = -FLT_MAX;
		int next = sorted_inum_[attr][nstart];
		float next_value = set_.get_attribute(next, attr);
		float cur_value;
		// Look for splits
		for (int i = nstart; i < nend - 1; ++i) {
			int cur = next;
			next = sorted_inum_[attr][ i + 1];
			if (SPLIT_METHOD == 0)
			{
				int label = int(set_.fgbglabel(cur));
				float weight = (*weight_list_)[cur];
				split_fgbgdist[kRight].remove(label, weight);
				split_fgbgdist[kLeft].add(label, weight);
			}
			else if (SPLIT_METHOD == 1)
			{
				int label = int(set_.viewpartlabel(cur));
				/*if (label != 0)
				{*/
					float weight = (*weight_list_)[cur];
					split_viewpartdist[kRight].remove(label, weight);
					split_viewpartdist[kLeft].add(label, weight);
				//}
			}
			cur_value =  next_value;
			next_value = set_.get_attribute(next, attr);

			if (cur_value < next_value) {
				// Calculate gain (can be sped up with incremental calculation!
				float split_entropy;
				if (SPLIT_METHOD == 0)
					split_entropy = DiscreteDist::entropy_conditioned(&split_fgbgdist[0], 2);
				else if (SPLIT_METHOD == 1)
					split_entropy = DiscreteDist::entropy_conditioned(&split_viewpartdist[0], 2);
				float curr_gain = prior_entropy - split_entropy;
				if (curr_gain > *best_gain) {
					*best_gain = curr_gain;
					*split_idx = i;
					*split_point = (cur_value + next_value)/2.0;
				}
			}
		}
		/*if (*split_idx < 0)
		{
			printf("%d %d %f", nstart, nend, cur_value);
			printf(" ");
		}*/

	}
	int compare (const void * a, const void * b)
	{
		return ( *(int*)a - *(int*)b );
	}

	void Tree::get_random_split_for_attr(tree_node* n,
		int attr,
		float prior_entropy,
		int* split_idx,
		float* split_point,
		float* best_gain)
	{
		//build random split nodes
		int * trialIndex;
		trialIndex = new int [iSplitNum_];
		for (int i = 0; i < iSplitNum_; i++)
		{
			trialIndex[i] = rand() % n->size + n->start;
		}
		qsort(trialIndex, iSplitNum_, sizeof(int), compare);

		int nstart = n->start;
		int nend = n->start + n->size;
		DiscreteDist split_dist[2];
		// Move all the instances into the right split at first
		for (int i = nstart; i < nend; ++i) {
			int inst_no = sorted_inum_[attr][i];
			split_dist[kRight].add(set_.fgbglabel(inst_no),
				(*weight_list_)[inst_no]);
		}
		// set up initial values
		*best_gain = -FLT_MAX;
		int next = nstart;
		float next_value = set_.get_attribute(sorted_inum_[attr][next], attr);

		// Look for splits
		for (int j = 0; j < iSplitNum_; ++j)
		{
			int cur = next;
			float cur_value = next_value;
			for (int k = cur; k < trialIndex[j]; k++)
			{
				int ii = sorted_inum_[attr][k];
				int label = int(set_.fgbglabel(ii));
				float weight = (*weight_list_)[ii];
				split_dist[kRight].remove(label, weight);
				split_dist[kLeft].add(label, weight);
			}

			next = sorted_inum_[attr][ trialIndex[j]];
			next_value = set_.get_attribute(next, attr);

			if (cur_value < next_value) {
				// Calculate gain (can be sped up with incremental calculation!
				float split_entropy = DiscreteDist::entropy_conditioned(split_dist, 2);
				float curr_gain = prior_entropy - split_entropy;
				if (curr_gain > *best_gain) {
					*best_gain = curr_gain;
					*split_idx = trialIndex[j];
					*split_point = (cur_value + next_value)/2.0;
				}
			}
		}
	}
	int Tree::predict(const InstanceSet& set, int instance_no, int * leaf_no, int *terminal) const {
		//base case
		bool result = false;
		int cur_node = 0;
		int label = 0;

		while (!result) {
			const tree_node* n = &vecnodes_.at(cur_node);
			assert(n->status == TERMINAL || n->status == SPLIT);
			if (n->status == TERMINAL) {
				result = true;
				label = n->fgbglabel;
				if (leaf_no != NULL){
					*leaf_no = n->leaf_no;
				}
			} else {
				if (set.get_attribute(instance_no, n->attr) < n->split_point) {
					cur_node = n->left;
				} else {
					cur_node = n->right;
				}
			}
		}
		if (terminal != NULL) {
			*terminal = cur_node;
		}
		return label;
	}

	int Tree::predictviewpart( const InstanceSet& set, int instance_no, int *leaf_no /*= NULL */, int *terminal /*= NULL*/ ) const
	{
		//base case
		bool result = false;
		int cur_node = 0;
		int label = 0;

		while (!result) {
			const tree_node* n = &vecnodes_.at(cur_node);
			assert(n->status == TERMINAL || n->status == SPLIT);
			if (n->status == TERMINAL) {
				result = true;
				label = n->viewpartlabel;
				if (leaf_no != NULL){
					*leaf_no = n->leaf_no;
				}
			} else {
				if (set.get_attribute(instance_no, n->attr) < n->split_point) {
					cur_node = n->left;
				} else {
					cur_node = n->right;
				}
			}
		}
		if (terminal != NULL) {
			*terminal = cur_node;
		}
		return label;
	}

	int Tree::predict_singlefeauture(float * feature, int * leaf_no, int *terminal) const {
		//base case
		bool result = false;
		int cur_node = 0;
		int label = 0;

		while (!result) {
			const tree_node* n = &vecnodes_.at(cur_node);
			assert(n->status == TERMINAL || n->status == SPLIT);
			if (n->status == TERMINAL) {
				result = true;
				label = n->fgbglabel;
				if (leaf_no != NULL){
					*leaf_no = n->leaf_no;
				}
			} else {
				if (feature[n->attr] < n->split_point) {
					cur_node = n->left;
				} else {
					cur_node = n->right;
				}
			}
		}
		if (terminal != NULL) {
			*terminal = cur_node;
		}
		return label;
	}


	//int Tree::predict_singlefeauture(float * feature, int * leaf_no, CFeatureExtractor & fe, int *terminal) const {
	//	//base case
	//	bool result = false;
	//	int cur_node = 0;
	//	int label = 0;

	//	while (!result)
	//	{
	//		const tree_node* n = &nodes_[cur_node];
	//		assert(n->status == TERMINAL || n->status == SPLIT);
	//		if (n->status == TERMINAL)
	//		{
	//			result = true;
	//			label = n->label;
	//			if (leaf_no != NULL){
	//				*leaf_no = n->leaf_no;
	//			}
	//		}
	//		else
	//		{
	//			if (feature[n->attr] == - FLT_MAX)
	//			{
	//				 feature[n->attr] = fe.GetCurFeatureByIndex(n->attr);
	//				 //printf("Compute the feature currently.\n" );
	//			}
	//			if (feature[n->attr] < n->split_point)
	//			{
	//				cur_node = n->left;
	//			} else
	//			{
	//				cur_node = n->right;
	//			}
	//		}
	//	}
	//	if (terminal != NULL) {
	//		*terminal = cur_node;
	//	}
	//	return label;
	//}

	void Tree::confusion_matrix() const{
		int pcorrect = 0;
		int pwrong = 0;
		int ncorrect = 0;
		int nwrong = 0;
		for (int i =0; i < set_.size(); ++i) {
			int iPredict = predict(set_, i);
			if (iPredict == set_.fgbglabel(i))
				pcorrect++;
			else
				pwrong++;
			/*if (set_.label(i) == 1)
			{
				if (iPredict == 1)
				{
					pcorrect++;
				}
				else
				{
					pwrong++;
				}
			}
			else
			{
				if (iPredict == 0)
				{
					ncorrect++;
				}
				else
				{
					nwrong++;
				}
			}*/
		}
		/*cout << "\tcomfusion_matrix:" << endl;
		cout << "\t\t" << ncorrect << "  " << nwrong<< endl;
		cout << "\t\t" << pwrong << "  " << pcorrect << endl;*/
		cout << "\t\t" << "accuracy:" << (float)(pcorrect)/(pcorrect+pwrong) << endl;

	}
	void Tree::training_accuracy( float * accuracy ) const
	{
		int correct = 0;
		for (int i = 0; i < set_.size(); ++i) {
			if (predict(set_, i) == set_.fgbglabel(i))
				correct++;
		}
		accuracy[0] = float(correct) / set_.size();

		correct = 0;
		int nfg = 0;
		for (int i = 0; i < set_.size(); ++i) {
			if (set_.fgbglabel(i) == 1)
			{
				nfg++;
				if (predictviewpart(set_, i) == set_.viewpartlabel(i))
					correct++;
			}
		}
		accuracy[1] = float(correct) / nfg;
	}

	void Tree::print() const {
		// Loop through active nodes
		cout << "\tmin_gain_split " << min_gain_split_ << endl;
		int cur_max_depth = -1;
		int cur_min_depth = INT_MAX;
		int cur_min_size = INT_MAX;
		int cur_max_size = INT_MIN;
		float cur_min_gain = 2.0f;
		float cur_max_gain = -0.1f;
		const int nSizeDist = 4;
		int nodesSizeDist[] = {21, 50, 100, 150};
		nodesSizeDist[0] = min_size_ + 1;
		int nodesSizeDistCount[nSizeDist + 1] = {0};

		const int nEntDist = 4;
		float nodesEntDist[] = {20.0f, 40.0f, 60.0f, 80.0f};
		nodesEntDist[0] = min_gain_split_ + 5.0f;
		int nodesEntDistCount[nEntDist + 1] = {0};

		for (int i = 0; i < vecnodes_.size(); ++i)
		{
			if (vecnodes_.at(i).status == TERMINAL)
			{
				if (vecnodes_.at(i).depth > cur_max_depth)
				{
					cur_max_depth = vecnodes_.at(i).depth;
				}
				if (vecnodes_.at(i).depth < cur_min_depth)
				{
					cur_min_depth = vecnodes_.at(i).depth;
				}
				if (vecnodes_.at(i).size > cur_max_size)
				{
					cur_max_size = vecnodes_.at(i).size;
				}
				if (vecnodes_.at(i).size < cur_min_size)
				{
					cur_min_size = vecnodes_.at(i).size;
				}
				int iD;
				for ( iD = 0; iD < nSizeDist; iD++)
				{
					if (vecnodes_.at(i).size <= nodesSizeDist[iD])
					{
						nodesSizeDistCount[iD] ++;
						break;
					}
				}
				if (iD == nSizeDist)
				{
					nodesSizeDistCount[nSizeDist] ++;
				}
			}
			if (vecnodes_.at(i).status == SPLIT)
			{
				if (vecnodes_.at(i).fgbgentropy > cur_max_gain)
				{
					cur_max_gain = vecnodes_.at(i).fgbgentropy;
				}
				if (vecnodes_.at(i).fgbgentropy < cur_min_gain)
				{
					cur_min_gain = vecnodes_.at(i).fgbgentropy;
				}
				int iD;
				for (iD = 0; iD < nEntDist; iD++)
				{
					if (vecnodes_.at(i).fgbgentropy <= nodesEntDist[iD])
					{
						nodesEntDistCount[iD] ++;
						break;
					}
				}
				if (iD == nEntDist)
				{
					nodesEntDistCount[nEntDist] ++;
				}
			}
		}
		cout << "\tTree with " << vecnodes_.size() << " nodes " << endl;
		cout << "\tSplit nodes: " << split_nodes_ << endl;
		cout << "\tTerminal nodes: " << terminal_nodes_ << endl;
		cout << "\tTree depth " << cur_min_depth + 1 << "--" << cur_max_depth + 1 << endl; //depth starts from 0
		cout << "\tTree node size: " << cur_min_size << "--" << cur_max_size << endl;
		cout << "\tTree node gain: " << cur_min_gain << "--" << cur_max_gain << endl;
		float accuracy[2];
		training_accuracy(accuracy);
		cout << "\tTraining acc: " << accuracy[0] << "/" << accuracy[1] << endl;
		cout << "\tInstance num: " << num_instances_ << endl;

		cout << "\tLeaf node size distribution:" << endl;
		//size
		for (int iD = 0; iD < nSizeDist; iD++)
		{
			cout << "\t\t <= " << nodesSizeDist[iD] << "    " << nodesSizeDistCount[iD] << endl;
		}
		cout << "\t\t > " << nodesSizeDist[nSizeDist - 1] << "    " << nodesSizeDistCount[nSizeDist] << endl;
		//entropy
		for (int iD = 0; iD < nEntDist; iD++)
		{
			cout << "\t\t <= " << nodesEntDist[iD] << "    " << nodesEntDistCount[iD] << endl;
		}
		cout << "\t\t > " << nodesEntDist[nEntDist - 1] << "    " << nodesEntDistCount[nEntDist] << endl;


		confusion_matrix();
	}

    //Msun added
	vector<int>* Tree::get_vecpatchIds( int terminialLocalId)
	{
//	    cout << TerminalNodeGlobalNodelIds_.size() << endl;
        int terminalGlobalId = TerminalNodeGlobalNodelIds_[terminialLocalId];
//        cout << vecnodes_.at(terminalGlobalId).leaf_no << endl;
//        cout << vecnodes_.at(terminalGlobalId).vecpatchid.at(0) <<" " <<
//            vecnodes_.at(terminalGlobalId).vecpatchid.at(1) << endl;
//        vecpatchIdptr = & vecnodes_.at(terminalGlobalId).vecpatchid;
//        cout << vecpatchIdptr->at(0) << " " << vecpatchIdptr->at(1) << endl;
        return &( vecnodes_.at(terminalGlobalId).vecpatchid);
    }
} //namespace
