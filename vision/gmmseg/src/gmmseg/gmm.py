import numpy
np = numpy
import prob
pb = prob
import fun
import random
import string
from itertools import *
from StringIO import StringIO

def distance_to(pt, points):
   """
   Deprecated: This could be implemented more efficiently.
   This returns the Euclidean distances from pt to points.
   """
   return numpy.power(
         numpy.sum(
            numpy.power(
               numpy.tile(pt, 
                  (1, points.shape[1])) - points, 2), 0), 0.5)



def simple_start(num_gaussians, points):
   """
   Returns simple initialization GMM. 
   """
   return rand_start(num_gaussians, points)


def far_start(num_gaussians, points, picked_so_far = None):
   """
   Returns a list of num_gaussians tuples, where each tuple has a prob.Gaussian, with covariance I and a mean greedly selected from points such that it is far away from the other means, and a weight. The output of this function can be used to initialize GMM. 
   """
   if (points.__class__ != numpy.matrix):
       raise RuntimeError("Param points is not of type matrix")
   def pt_tuple(i):
      pt = points[:,i]
      gauss = prob.Gaussian(pt, numpy.eye(points.shape[0]))
      distances = distance_to(pt, points)
      return (pt, gauss, distances)

   if (picked_so_far == None):
      i = random.randint(0, points.shape[1]-1)
      pt, gauss, distances = pt_tuple(i)
      if (num_gaussians == 1):
         return [gauss]
      else:
         return far_start(num_gaussians - 1, points, [(pt, gauss, distances)])
   else:
      #Pick the point that is the furthest away from all existing points
      #sum up all existing distances
      def vector_add (d1, d2): return d1+d2
      dist_pick = reduce(vector_add, [d for p,g,d in picked_so_far])
      max_index = numpy.argmax(dist_pick)

      #append picked point and calculate its distances
      pt, gauss, distances = pt_tuple(max_index)
      if (num_gaussians == 1):
         total_num = len(picked_so_far) + 1.0
         weight = 1.0 / total_num
         return [(pdf, weight) for p,pdf,d in picked_so_far] + [(gauss, weight)] 
      else:
         p = picked_so_far + [(pt, gauss, distances)]
         return far_start(num_gaussians - 1, points, p)


def rand_start(num_gaussians, points):
   """
   Returns a list of num_gaussians tuples, where each tuple has a prob.Gaussian, with covariance I and a mean equal to a randomly chosen point from points, and a weight chosen to sum to one. The output of this function can be used to initialize GMM. 
   """
   if (points.__class__ != numpy.matrix):
       raise RuntimeError("Param points is not of type matrix")

   #Pick n random gaussians (don't do this)
   num_dim = points.shape[0]
   num_pts = points.shape[1]
   if num_pts < num_gaussians:
       raise RuntimeError("Not enough points")

   weight  = 1.0 / num_gaussians
   start_gaussians = [(prob.Gaussian(points[:,rint], numpy.eye(num_dim)), weight)
                      for rint in 
                      fun.truncate(fun.repeat(random.randint,(0,num_pts-1)),num_gaussians)]
   return start_gaussians



class GMM(object):
   """
   Gaussian mixture model (GMM). This can be used to fit a Gaussian mixture model to data, and more!
   """

   def __init__(self, points, start_gaussians):
      """
      Create a Gaussian mixture model (GMM) object using:
      + points : the data points to which the GMM will be fit. points must be a matrix containing the input data points.
      + start_gaussians : the initial state of the GMM, which consists of a list of tuples, where each tuple has the form (prob.Gaussian, weight) and the weights sum to 1. 
      """
      if (points.__class__ != numpy.matrix):
         raise RuntimeError("Param points is not of type matrix")
      self.points = points
      self.start_gaussians = start_gaussians
      self.gaussians = self.start_gaussians
      self.num_gaussians = len(self.start_gaussians)
      self.num_pts = self.points.shape[1]
      self.gamma = None

   def pdf(self):
      """
      Partially applied gmm 
      """
      gpdfs = [(g.pdf(),w) for g,w in self.gaussians]
      def gmm_pdf(x):
         vals = [pdf(x) * weight for pdf, weight in gpdfs]
         return reduce(add,vals)
      return gmm_pdf


   def pdf_mat(self):
      """
      Return a partially applied Gaussian Mixture Model pdf that takes in a matrix whose columns are the input vectors.
      """
      gpdfs = [(g.pdf_mat(),w) for g,w in self.gaussians]
      def gmm_pdf_mat(x):
         """Partially applied Gaussian Mixture Model pdf that takes in a matrix whose columns are the input vectors"""
         vals = [pdf(x) * weight for pdf, weight in gpdfs]
         return reduce(add,vals)
      return gmm_pdf_mat


   def membership_mat(self):
      """
      Return a function that returns the membership probabilities of an input matrix (columns are vectors) (output has the probability that each of the mixed Gaussians would have generated the vector). This includes the mixing parameter.
      """
      gpdfs = [(g.pdf_mat(),w) for g,w in self.gaussians]
      def member_mat(x):
         vals = [pdf(x) * weight for pdf, weight in gpdfs]
         return numpy.vstack(vals)
      return member_mat

   def __str__(self):
       p = StringIO()
       for g,w in self.gaussians:
           print >>p, "weight: ", w, "\ngaussian: ", g
       return p.getvalue()


   def membership(self):
      """
      Return a function that returns the membership probabilities of an input vector (the probability that each of the mixed Gaussians would have generated the vector). This includes the mixing parameter.
      """
      gpdfs = [(g.pdf(),w) for g,w in self.gaussians]
      def member(x):
         vals = [pdf(x) * weight for pdf, weight in gpdfs]
         return vals
      return member

   def __str__(self):
       p = StringIO()
       for g,w in self.gaussians:
           print >>p, "weight: ", w, "\ngaussian: ", g
       return p.getvalue()


   #E Step: calculate responsibilities, gamma
   def E(self):
      """
      Performs E step of the EM algorithm. The EM algorithm finds locally optimal parameters for the Gaussian mixture model, given the data.
      """
      pdfs   = [(gaussian.pdf(), weight) for gaussian, weight in self.gaussians]
      self.gamma  = numpy.matrix(numpy.zeros((self.num_gaussians, self.num_pts)))
      for i_pt in xrange(self.num_pts):
         i_pt_total = 0.0
         #Calculate assignment probabilities
         for j_pdf, weighted_pdf in enumerate(pdfs):
            pdf_func, weight   = weighted_pdf
            i_prob             = weight * pdf_func(self.points[:,i_pt]) 
            i_pt_total        += i_prob
            self.gamma[j_pdf, i_pt] = i_prob

         #Normalize assignment probabilities
         if (i_pt_total == 0.0):
            #If the i_th point is too far away from all given gaussians
            #reset its assignment probabilities to be uniform
            self.gamma[:,i_pt] = 1.0 / self.num_gaussians
         else:
            self.gamma[:,i_pt] = self.gamma[:,i_pt] / i_pt_total


   #E Step: calculate responsibilities, gamma
   def E_mat(self):
      """
      Performs a fast matrix version of the E step of the EM algorithm. The EM algorithm finds locally optimal parameters for the Gaussian mixture model, given the data.
      """
      membership = self.membership_mat()
      probs = membership(self.points)
      pt_total = probs.sum(axis=0)
      #If the a point is too far away from all given gaussians
      #reset its assignment probabilities to be uniform
      #(avoids divide by zero...)
      pt_total[pt_total <= 0.0] = 1.0/self.num_gaussians
      self.gamma = probs/pt_total


   #M Step: estimate new Gaussians using normalized weights (gamma)
   def M(self):
      """
      Performs M step of the EM algorithm. The EM algorithm finds locally optimal parameters for the Gaussian mixture model, given the data.
      """
      def f(index):
         assignment_probs = numpy.matrix(self.gamma[index,:])
         weight           = numpy.sum(assignment_probs) / float(self.num_pts)
         estimated        = prob.fit(self.points, assignment_probs)
         #Prevent numerically unstable covariances
         estimated.cov    = estimated.cov + (0.001 * numpy.eye(estimated.cov.shape[0]))
         return (estimated, weight)
      self.gaussians = map(f, xrange(self.num_gaussians))


   #M Step: estimate new Gaussians using normalized weights (gamma)
   def M_mat(self):
      """
      Performs a fast matrix version of the M step of the EM algorithm. The EM algorithm finds locally optimal parameters for the Gaussian mixture model, given the data.
      """
      def f(index):
         assignment_probs = numpy.matrix(self.gamma[index,:])
         weight           = numpy.sum(assignment_probs) / float(self.num_pts)
         estimated        = prob.fit(self.points, assignment_probs)
         #Prevent numerically unstable covariances
         estimated.cov    = estimated.cov + (0.001 * numpy.eye(estimated.cov.shape[0]))
         return (estimated, weight)
      self.gaussians = map(f, xrange(self.num_gaussians))


   def fit_iteration(self):
      """
      Performs a single iteration of the EM algorithm
      """
      #self.E()
      self.E_mat()
      self.M()


   def fit(self, epsilon = .01, iter_limit=500):
      """Fits the Gaussian mixture model to the data using the EM algorithm. Fitting iterates until the error of the fit is below epsilon or the the number of iterations has reached iter_limit."""
      #Loop until convergence
      self.prev_gaussians = self.gaussians
      diff = 0

      def gauss_diff(a,b):
         g1, w1 = a
         g2, w2 = b
         return g1 - g2

      for num_iter in xrange(iter_limit):
         self.fit_iteration()
         diff = numpy.matrix([gauss_diff(a,b) for a,b in zip(self.prev_gaussians,
                                                             self.gaussians)]).T
         if (num_iter % 5)==0:
            print 'iteration ', num_iter
            #print diff
         if ((diff < epsilon).all()):
            return self
         else:
            self.prev_gaussians = self.gaussians

      return self



if __name__ == '__main__':
    """
    Example code
    """
    
    gm1 = np.matrix([[1.0, 1.0, 1.0]])
    gv1 = np.matrix([[ 0.5, 0.0, 0.0],
                     [ 0.0, 0.5, 0.0],
                     [ 0.0, 0.0, 0.5]])
    
    gm2 = np.matrix([[0.0, 0.0, 0.0]])
    gv2 = np.matrix([[ 0.5, 0.0, 0.0],
                     [ 0.0, 0.5, 0.0],
                     [ 0.0, 0.0, 0.5]])
    
    g1 = pb.Gaussian(gm1, gv1)
    g2 = pb.Gaussian(gm2, gv2)

    print 'g1 =', g1
    print
    print 'g2 =', g2
    print

    nsamp = 1000
    
    g1_samples = np.reshape(np.array([g1.sample() for i in range(nsamp)]),
                            [nsamp, 3])
    g2_samples = np.reshape(np.array([g2.sample() for i in range(nsamp)]),
                            [nsamp, 3])
    
    ## combine all the samples
    all_samples =  np.vstack((g1_samples, g2_samples))

    points = np.matrix(all_samples).T

    print 'number of dimensions for each point =', gm1.shape[1]
    print 'number of points =', nsamp
    print 'points.shape =', points.shape
    print 'points[:, :4] =', points[:,:4]
    print
    
    gmm_init = simple_start(2, points)

    gmm = GMM(points, gmm_init)
    
    print 'gmm =', gmm
    print

    print 'fitting gmm to points'
    print 'gmm.fit(epsilon=0.001, iter_limit=1000)'
    gmm.fit(epsilon=0.001, iter_limit=1000)
    print
    
    print 'result of fitting'
    print 'gmm =', gmm
