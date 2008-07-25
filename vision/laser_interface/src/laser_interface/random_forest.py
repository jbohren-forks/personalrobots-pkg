import numpy as np
import itertools as it
import functools as ft

class Dataset:
    def __init__(self, inputs, outputs):
        """
            inputs coded as numpy array, column vectors
            outputs also as numpy array, column vectors
        """
        self.inputs  = inputs
        self.outputs = outputs
        assert(inputs.shape[1] == outputs.shape[1])

    def num_examples(self):
        return self.inputs.shape[1]

    def num_attributes(self):
        return self.inputs.shape[0]

    def split_continuous(self, attribute, split_point):
        selected_attribute = self.inputs[attribute, :]
        leq_bool           = selected_attribute <= split_point
        _, leq_col         = np.where(leq_bool)

        #print 'leq_col', leq_col
        if leq_col.shape[1] > 0:
            leq_dataset        = Dataset(self.inputs[:, leq_col.A[0]], self.outputs[:, leq_col.A[0]])
        else:
            leq_dataset        = Dataset(np.matrix([]), np.matrix([]))

        _, gt_col          = np.where(~leq_bool)
        if gt_col.shape[1] > 0:
            gt_dataset         = Dataset(self.inputs[:, gt_col.A[0]], self.outputs[:, gt_col.A[0]])
        else:
            gt_dataset         = Dataset(np.matrix([]), np.matrix([]))
        
        ret_sets = []
        if leq_dataset.num_examples() > 0:
            ret_sets.append(leq_dataset)
        if gt_dataset.num_examples() > 0:
            ret_sets.append(gt_dataset)
        return ret_sets

    def unique_values(self, attribute_number, set='input'):
        if set == 'input':
            examples = self.inputs
        else:
            examples = self.outputs

        values   = dict()
        for instance_idx in xrange(examples.shape[1]):
            values[examples[attribute_number, instance_idx]] = True
        k = values.keys()
        k.sort()
        return k

    def bootstrap_samples(self, number_samples, points_per_sample):
        for i in xrange(number_samples):
            selected_pts     = np.random.randint(0, self.inputs.shape[1], points_per_sample)
            selected_inputs  = self.inputs[:, selected_pts]
            selected_outputs = self.outputs[:, selected_pts]
            print 'Dataset.bootstrap count', i
            yield Dataset(selected_inputs, selected_outputs)

    def entropy_discrete(self):
        values = self.unique_values(0, 'output')
        #print 'entropy_discrete: values', values
        #for each output class calculate
        def calc_class_entropy(value):
            number_in_class     = np.sum(self.outputs[0,:] == value)
            percentage_in_class = (number_in_class / float(self.num_examples()))
            return -percentage_in_class * np.log2(percentage_in_class)
        return np.sum(map(calc_class_entropy, values))

    def append(self, another_dataset):
        self.inputs  = np.concatenate((self.inputs, another_dataset.inputs), axis=1)
        self.outputs = np.concatenate((self.outputs, another_dataset.outputs), axis=1)

class LinearDimReduceDataset(Dataset):
    def __init__(self, inputs, outputs):
        Dataset.__init__(self, inputs, outputs)

    def set_projection_vectors(self, vec):
        '''
            projection vectors are assumed to be columnwise
        '''
        self.projection_basis = vec

    def reduce(self, data_points):
        return self.projection_basis.T * data_points

    def reduce_input(self):
        '''
           reduce dimensionality of this dataset
        '''
        self.inputs =  self.projection_basis.T * self.inputs

def binary_less_than(attribute, threshold, input_vec):
    return input_vec[attribute,0] <= threshold

def binary_greater_than(attribute, threshold, input_vec):
    return input_vec[attribute,0] > threshold

def create_binary_tests(attribute, threshold):
    return [ft.partial(binary_less_than, attribute, threshold), 
            ft.partial(binary_greater_than, attribute, threshold)]

def mode_exhaustive(set):
    '''
       Finds the mode of a given set
    '''

    #Count, store in dictionary
    mdict = dict()
    for s in set:
        has_stored = False
        for k in mdict.keys():
            if k == s:
                mdict[k] = 1+mdict[k]
                has_stored = True
        if not has_stored:
            mdict[s] = 1

    #Find the key with maximum votes
    max_key   = None
    max_count = -1
    for k in mdict.keys():
        if mdict[k] > max_count:
            max_key   = k
            max_count = mdict[k]
    #print 'mode_exhaustive: ', mdict
    return max_key, mdict

def min_entropy_split(dataset):
    '''
        Find the split that produces subsets with the minimum combined entropy
        return splitting attribute & splitting point for that attribute
    '''
    #print 'in min_entropy_split'
    # Assume inputs are continuous, and are column vectors.
    hypotheses     = []
    entropies      = []
    # For each attribute find the best split point.
    for attribute in xrange(dataset.num_attributes()):
        values = dataset.unique_values(attribute)
        #Iterate over the possible values of split & calculate entropy for each split.
        for split_point in values:
            def calc_entropy(data_set):
                num_points = data_set.num_examples()
                return (num_points / float(dataset.num_examples())) * data_set.entropy_discrete()
            split_entropy = map(calc_entropy, dataset.split_continuous(attribute, split_point))
            hypotheses.append((attribute, split_point))
            entropies.append(sum(split_entropy))
    # Select the attribute split pair that has the lowest entropy.
    entropies                              = np.matrix(entropies)
    min_idx                                = np.argmin(entropies)
    return hypotheses[min_idx]

def random_subset(subset_size, total_size):
    #print 'in random_subset'
    assert(subset_size <= total_size)
    occupancy = np.matrix(np.zeros((1, total_size)))
    while occupancy.sum() < subset_size:
        occupancy[0, np.random.randint(0, total_size)] = 1
    rows, columns = np.where(occupancy > 0)
    return columns.A[0]

def split_random_subset(subset_size, total_size):
    assert(subset_size <= total_size)
    occupancy = np.matrix(np.zeros((1, total_size)))
    while occupancy.sum() < subset_size:
        occupancy[0, np.random.randint(0, total_size)] = 1
    bool_sel                = occupancy > 0
    rows, columns_subset    = np.where(bool_sel)
    rows, columns_remaining = np.where(np.invert(bool_sel))
    return columns_subset.A[0], columns_remaining.A[0]

def random_subset_split(num_subset, dataset):
    ''' splitter in decision tree '''
    #print 'in random_subset_split'
    #print 'num_subset', num_subset, dataset, 'dataset.input.shape', dataset.inputs.shape
    subset_indexes   = random_subset(num_subset, dataset.num_attributes())
    sub_dataset      = Dataset(dataset.inputs[subset_indexes,:], dataset.outputs)
    attribute, point = min_entropy_split(sub_dataset)
    return subset_indexes[attribute], point

def totally_random_split(dataset):
    #print 'totally random'
    attr     = np.random.randint(0, dataset.num_attributes())
    split_pt = dataset.inputs[attr, np.random.randint(0, dataset.num_examples())]
    return attr, split_pt

class DecisionTree:
    def __init__(self, dataset=None, splitting_func=min_entropy_split):
        self.children   = None
        self.prediction = None
        if dataset is not None:
            self.train(dataset, splitting_func=splitting_func)

    def train(self, dataset, splitting_func=min_entropy_split):
        if not self.make_leaf(dataset):
            #print 'in train.splitting', dataset.num_examples()
            self.split_attribute, self.split_point = splitting_func(dataset)
            #print 'self.split_attribute, self.split_point', self.split_attribute, self.split_point 
            data_sets                              = dataset.split_continuous(self.split_attribute, self.split_point)
            if len(data_sets) < 2:
                self.prediction = dataset.outputs
                return
            
            def tree_split(set):
                #print 'tree', set.num_examples()
                return DecisionTree(set, splitting_func=splitting_func)
            # Create & train child decision nodes
            tests            = create_binary_tests(self.split_attribute, self.split_point)
            self.children    = zip(tests, map(tree_split, data_sets))

    def make_leaf(self, dataset):
        if np.all(dataset.outputs[:,0] == dataset.outputs):
            self.prediction = dataset.outputs[:,0]
            #print 'leaf'
            return True
        elif np.all(dataset.inputs[:,0] == dataset.inputs):
            self.prediction = dataset.outputs
            #print 'leaf'
            return True
        else:
            return False

    def predict(self, input):
        if self.prediction is not None:
            return self.prediction[:, np.random.randint(0, self.prediction.shape[1])]
        else:
            for test, child in self.children:
                if test(input):
                    return child.predict(input)
            raise RuntimeError("DecisionTree: splits not exhaustive, unable to split for input" + str(input.T))

class RFBase:
    def __init__(self, dataset=None, number_of_dimensions=None, number_of_learners=100):
        """ 
            number_of_dimensions   unclear which direction, but should be around 10-20% of original 
                                   data dimension
            number_of_learners        limited by processor performance, higher is better
        """
        self.number_of_learners   = number_of_learners
        self.number_of_dimensions = number_of_dimensions
        if dataset != None:
            self.train(dataset)

    def predict(self, data, vote_combine_function=None):
        def predict_(learner):
            return learner.predict(learner.transform_input(data))
        predictions = map(predict_,self.learners)
        if vote_combine_function is not None:
            return vote_combine_function(predictions)
        else:
            return mode_exhaustive(predictions)

    def train(self, dataset):
        pass

def identity(x):
    return x

class RFBreiman(RFBase):
    def train(self, dataset):
        def train_trees(examples_subset):
            tree = DecisionTree()
            #tree.train(examples_subset, splitting_func=ft.partial(random_subset_split, self.number_of_dimensions))
            tree.train(examples_subset, splitting_func=totally_random_split)
            #use identity function
            tree.transform_input = identity 
            return tree

        if self.number_of_dimensions == None:
            self.number_of_dimensions = min(np.log2(dataset.num_attributes()) + 1, 1)
        points_per_sample = dataset.num_examples() * 1.0 / 3.0
        self.learners     = map(train_trees, dataset.bootstrap_samples(self.number_of_learners, points_per_sample))

class RFRandomInputSubset(RFBase):
    def train(self, dataset):
        def train_trees(examples_subset):
            #select a subset of dimensions
            dims               = random_subset(self.number_of_dimensions, examples_subset.num_attributes())
            subset_input       = examples_subset.inputs[dims, :]
            reduced_sample     = Dataset(subset_input, examples_subset.outputs)
            tree               = DecisionTree(reduced_sample)
            tree.dimensions_subset = dims
            def transform_input(input):
                return input[dims, :]
            tree.transform_input = transform_input
            return tree

        if self.number_of_dimensions == None:
            self.number_of_dimensions = min(np.log2(dataset.num_attributes()) + 1, 1)
        points_per_sample = dataset.num_examples() * 1.0 / 3.0
        self.learners     = map(train_trees, dataset.bootstrap_samples(self.number_of_learners, points_per_sample))

def evaluate_classifier(building_func, data, times=10.0, percentage=None, extra_args={}):
    total_pts  = data.num_examples()
    for i in range(times):
        if percentage == None:
            percentage = (i+1)/times
        num_examples = int(round(total_pts*percentage))
        print 'Evaluate classifier built with', percentage*100, '% data, num examples', num_examples
        subset, unselected = split_random_subset(num_examples, total_pts)
        i                  = data.inputs[:,  subset]
        o                  = data.outputs[:, subset]
        print "Building classifier..."
        classifier         = building_func(Dataset(i,o), **extra_args)
        print "done building..."

        confusion_matrix   = dict()
        count              = np.matrix(np.zeros((total_pts, 1)))
        print 'Total points', total_pts
        for i in xrange(total_pts):
            prediction = classifier.predict(data.inputs[:,i])
            true_val   = data.outputs[:,i]
            #print '       prediction', prediction, ' actual', true_val
            if prediction == true_val:
                count[i, 0] = 1
            else:
                count[i, 0] = 0
            if confusion_matrix.has_key(true_val[0,0]):
                if confusion_matrix[true_val[0,0]].has_key(prediction[0,0]):
                    confusion_matrix[true_val[0,0]][prediction[0,0]] = confusion_matrix[true_val[0,0]][prediction[0,0]] + 1
                else:
                    confusion_matrix[true_val[0,0]][prediction[0,0]] = 1
            else:
                confusion_matrix[true_val[0,0]] = dict()
                confusion_matrix[true_val[0,0]][prediction[0,0]] = 1

        print 'Percent Correct', (np.sum(count) / float(total_pts)) * 100.0, '%'
        print '    on training set', 100.0 * np.sum(count[subset, 0]) / float(len(subset)), '%'
        print '    on testing set',  100.0 * np.sum(count[unselected, 0]) / float(len(unselected)), '%'
        print 'Confusion'
        for k in confusion_matrix.keys():
            sum = 0.0
            for k2 in confusion_matrix[k]:
                sum = sum + confusion_matrix[k][k2]
            for k2 in confusion_matrix[k]:
                print 'true class', k, 'classified as', k2, 100.0 * (confusion_matrix[k][k2] / sum), '% of the time'


if __name__ == '__main__':
    print "Test Dataset.entropy_discrete"
    input = np.matrix([1, 0, 2, 3])
    output = np.matrix(['yes', 'no', 'yes', 'no'])
    dataset = Dataset(input, output)
    print "Discrete entropy for", output
    print dataset.entropy_discrete()

    print "Test Dataset.bootstrap_samples"
    input = np.matrix([1, 2, 3, 4, 5, 6, 7, 8])
    output = np.matrix(['yes', 'no', 'yes', 'no', 'yes', 'no', 'yes', 'no'])
    dataset = Dataset(input, output)
    samples = dataset.bootstrap_samples(3, 4)
    for idx, s in enumerate(samples):
        print idx, s.inputs

    print "Test Dataset.unique_values"
    print dataset.unique_values(0)

    #Setup for repeated testing
    iris_array = np.matrix(np.loadtxt('iris.data', dtype='|S30', delimiter=','))
    inputs     = np.float32(iris_array[:, 0:4]).T
    outputs    = iris_array[:, 4].T
    dataset    = Dataset(inputs, outputs)

    print '================================'
    print "Test DecisionTree"
    evaluate_classifier(DecisionTree, dataset, 1, .7)

    print '================================'
    #print "Test random forest"
    #for i in range(4):
    #    #print "Test RFRandomInputSubset"
    #    #evaluate_classifier(RFRandomInputSubset, dataset, 1, .7)
    #    print "Test RFBreiman"
    #    evaluate_classifier(RFEntropySplitRandomInputSubset, dataset, 1, .7)

    import laser_detector as ld
    dataset = ld.load_pickle('PatchClassifier.dataset.pickle')
    print '==============================================================='
    print '==============================================================='
    print "Test RFBreiman"
    for i in range(4):
        print 'using', (i+1)*10, 'trees'
        evaluate_classifier(RFBreiman, dataset, 1, .9, extra_args={'number_of_learners': (i+1)*10})
    print '==============================================================='
    print '==============================================================='
    print "Test RFRandomInputSubset"
    evaluate_classifier(RFRandomInputSubset, dataset, 1, .9)






