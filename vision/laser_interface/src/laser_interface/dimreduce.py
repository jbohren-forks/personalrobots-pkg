import numpy as np
#from pylab import *

def pca_gain_threshold(s, percentage_change_threshold=.15):
    if s.__class__ != np.ndarray:
        raise ValueError('Need ndarray as input.') 
    shifted      = np.concatenate((s[1:].copy(), np.array([s[-1]])), axis=1)
    diff         = s - shifted
    percent_diff = diff / s
    positions    = np.where(percent_diff < percentage_change_threshold)
    return positions[0][0]

def pca_variance_threshold(eigen_values, percent_variance=.9):
    eigen_sum    = np.sum(eigen_values)
    #print 'pca_variance_threshold: eigen_sum', eigen_sum
    eigen_normed = np.cumsum(eigen_values) / eigen_sum
    positions    = np.where(eigen_normed > percent_variance)
    print 'pca_variance_threshold: percent_variance', percent_variance
    #print positions
    return positions[0][0]

def pca(data):
    cov_data = np.cov(data)
    u, s, vh = np.linalg.svd(cov_data)
    return u,s,vh

def pca_vectors(data, percent_variance):
    u, s, vh = pca(data)
    number_of_vectors = pca_variance_threshold(s, percent_variance=percent_variance)
    return np.matrix(u[:,0:number_of_vectors+1])

def randomized_vectors(dataset, number_of_vectors):
    rvectors = np.matrix(np.random.random_sample((dataset.num_attributes(), number_of_vectors))) * 2 - 1.0
    lengths  = np.diag(1.0 / np.power(np.sum(np.power(rvectors, 2), axis=0), 0.5))
    return rvectors * lengths


