import random
import vop
import votools
from stereo_utils import camera

def vop3(L):
  """ Turn a list of triplets into a triplet of vop arrays """
  x0 = vop.array([ x for (x,_,_) in L ])
  y0 = vop.array([ y for (_,y,_) in L ])
  z0 = vop.array([ z for (_,_,z) in L ])
  return (x0, y0, z0)

def xform(M, x, y, z):
  """ Transform point (x,y,z) by matrix M.  Use vop's mad (multiply-add) operator for efficiency """
  nx = vop.mad(M[0], x, vop.mad(M[1], y, vop.mad(M[2], z, M[3])))
  ny = vop.mad(M[4], x, vop.mad(M[5], y, vop.mad(M[6], z, M[7])))
  nz = vop.mad(M[8], x, vop.mad(M[9], y, vop.mad(M[10], z, M[11])))
  return (nx, ny, nz)

class PoseEstimator:

  def __init__(self, *camparams):
    self.iet = 3.0
    self.setNumRansacIterations(100)
    if camparams != ():
      self.default_camera = camera.Camera(camparams)
    else:
      self.default_camera = None
    self.inl = []

  def setInlierErrorThreshold(self, t):
    self.iet = t

  def setNumRansacIterations(self, i):
    self.ransac_iterations = i
    self.r0 = vop.array([ random.random() for i in range(self.ransac_iterations) ])
    self.r1 = vop.array([ random.random() for i in range(self.ransac_iterations) ])
    self.r2 = vop.array([ random.random() for i in range(self.ransac_iterations) ])

  def inliers(self):
    return self.inl

  def estimate(self, cp1, cp0, pairs, polish = True):
    return self.estimateC(self.default_camera, cp0, self.default_camera, cp1, pairs, polish)

  def estimateC(self, cam1, cp1, cam0, cp0, pairs, polish = True):
    """
    Pose Estimator.  Given uvd points and cameras for two poses, along
    with a list of matching point pairs, returns the relative pose
    """

    if len(pairs) < 3:
      return (0, None, None)

    # Compute the arrays for xyz for RANSAC sampling.
    (p0_x, p0_y, p0_z) = cam0.pix2cam(*vop3(cp0))
    (p1_x, p1_y, p1_z) = cam1.pix2cam(*vop3(cp1))

    # Compute the paired arrays (x0,y0,z0) and (u1,v1,d1) for RANSAC confirmation
    p0 = zip(p0_x, p0_y, p0_z)
    x0,y0,z0 = vop3([ p0[i] for (i,j) in pairs ])
    u1,v1,d1 = vop3([ cp1[j] for (i,j) in pairs ])

    # Generate the random triplets.  Fiddling here is to do the "without
    # replacement" picks.  Formally (pick0[i] != pick1[i] != pick2[i])
    np = len(pairs)
    pick0 = vop.floor(self.r0 * np)
    pick1 = vop.floor(self.r1 * (np-1))
    pick1 = vop.where(pick1 < pick0, pick1, pick1 + 1)
    pick2 = vop.floor(self.r2 * (np-2))
    pick2 = vop.where(pick2 < vop.minimum(pick0,pick1), pick2, pick2 + 1)
    pick2 = vop.where(pick2 < vop.maximum(pick0,pick1), pick2, pick2 + 1)

    # Keep track of current best guess
    best = (0, None, None)
    best_inl = []

    for ransac in range(self.ransac_iterations):
      #triple = random.sample(pairs, 3)
      triple = ( pairs[int(pick0[ransac])], pairs[int(pick1[ransac])], pairs[int(pick2[ransac])] )

      # Find a pair of xyzs, then supply them to SVD to produce a pose
      ((a,_),(b,_),(c,_)) = triple
      p0s = [ p0_x[a], p0_x[b], p0_x[c],
              p0_y[a], p0_y[b], p0_y[c],
              p0_z[a], p0_z[b], p0_z[c] ]

      ((_,aa),(_,bb),(_,cc)) = triple
      p1s = [ p1_x[aa], p1_x[bb], p1_x[cc],
              p1_y[aa], p1_y[bb], p1_y[cc],
              p1_z[aa], p1_z[bb], p1_z[cc] ]

      def toofar(d0, d1):
        return d0==0 or d1==0 or (d1/d0)>1.1 or (d0/d1)>1.1

      if False:
        # Rufus: Check if there is any scale change between the pairs
        p0s_dist_ab = (p0_x[a]-p0_x[b])**2 + (p0_y[a]-p0_y[b])**2 + (p0_z[a]-p0_z[b])**2
        p1s_dist_ab = (p1_x[aa]-p1_x[bb])**2 + (p1_y[aa]-p1_y[bb])**2 + (p1_z[aa]-p1_z[bb])**2

        if toofar(p1s_dist_ab, p0s_dist_ab):
          continue

        p0s_dist_cb = (p0_x[c]-p0_x[b])**2 + (p0_y[c]-p0_y[b])**2 + (p0_z[c]-p0_z[b])**2
        p1s_dist_cb = (p1_x[cc]-p1_x[bb])**2 + (p1_y[cc]-p1_y[bb])**2 + (p1_z[cc]-p1_z[bb])**2

        if toofar(p1s_dist_cb, p0s_dist_cb):
          continue

        p0s_dist_ac = (p0_x[a]-p0_x[c])**2 + (p0_y[a]-p0_y[c])**2 + (p0_z[a]-p0_z[c])**2
        p1s_dist_ac = (p1_x[aa]-p1_x[cc])**2 + (p1_y[aa]-p1_y[cc])**2 + (p1_z[aa]-p1_z[cc])**2

        if toofar(p1s_dist_ac, p0s_dist_ac):
          continue

      R,T,RT = votools.SVD(p0s, p1s)
      #R,T,RT = votools.SVDe(p0s, p1s)

      # Check inliers for RT: xyz0 -> uvd0 vs uvd1
      (u0,v0,d0) = cam0.cam2pix(*xform(RT, x0, y0, z0))
      pred_inl = vop.where(vop.maximum(vop.maximum(abs(u0 - u1), abs(v0 - v1)), abs(d0 - d1)) > self.iet, 0.0, 1.0)
      inliers = int(pred_inl.sum())
      if inliers > best[0]:
        best = (inliers, R, T)
        best_inl = pred_inl

    self.inl = [ P for (P,F) in zip(pairs, best_inl) if F ]
    if polish and (best[0] > 6):
      uvds0Inlier = [ cp0[a] for (a,_) in self.inl ]
      uvds1Inlier = [ cp1[b] for (_,b) in self.inl ]
      carttodisp = cam0.cart_to_disp()
      disptocart = cam1.disp_to_cart()
      (inliers,R,T) = best
      (R, T) = votools.polish(uvds0Inlier, uvds1Inlier, carttodisp, disptocart, R, T)
      best = (inliers, R, T)

    return best
