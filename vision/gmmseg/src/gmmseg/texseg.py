from optparse import OptionParser
from opencv import cv
from opencv import highgui
import scipy.ndimage as ni
import Image
import ImageDraw
import util as ut
import numpy as np
import gmm as gm
import prob as pb
import itertools as it
rn = np.random
la = np.linalg

import psyco
psyco.full()


def gmm2ellipses(gmm):
    """Assumes that the first two dimensions of the gaussians are the spatial dimensions to be used."""
    ellipses = []
    for i, (g, w) in enumerate(gmm.gaussians):
        e = ut.Ellipse()
        e.set_from_gaussian(g,w)
        ellipses.append(e)
    return ellipses

        

def largest_region(binary_im):
    """
    Given a binary image this finds the connected component with the largest area. It returns a tuple (labels, size). labels is a binary image with the largest connected component set to 1 and everything else set to 0. size is the size of this region in pixels.
    """
    labels, n_labels = ni.label(binary_im)
    biggest = 0
    if n_labels == 0:
        return labels, 0
    
    if n_labels > 1:
        sizes = ni.sum(binary_im, labels, index=range(n_labels+1))
        biggest = np.argmax(sizes)
        size = sizes[biggest]
    elif n_labels == 1:
        biggest = 1
        size = binary_im.sum()
    else:
        raise ValueError('n_labels < 0!')

    labels[labels!=biggest] = 0
    labels[labels==biggest] = 1
    return labels, size 


def big_center_region(binary_im):
    """
    Given a binary image this finds the that is both large and close to the center of the image. It returns a tuple (labels, size). labels is a binary image with the selected connected component set to 1 and everything else set to 0. size is the size of this region in pixels.
    """
    labels, n_labels = ni.label(binary_im)
    biggest = 0
    if n_labels == 0:
        return labels, 0
    
    if n_labels > 1:
        sizes = ni.sum(binary_im, labels, index=range(n_labels+1))
        sizes = sizes #[1:]
        centers = ni.center_of_mass(binary_im, labels, index=range(n_labels+1))
        centers[0] = [480,640]
        centers = np.array(centers)
        h = binary_im.shape[0]
        w = binary_im.shape[1]
        im_center = np.array([h/2.0, w/2.0])
        distances = np.square(centers - im_center)
        distances = np.sqrt(distances.sum(axis=1))

        if True:
            min_diameter = 10 #pixels
            min_size = np.square(min_diameter)
            big_enough_mask = sizes > min_size

            max_dist = 200 #pixels
            close_enough_mask = distances < max_dist

            if False:
                top_n = 3
                size_sorter = np.argsort(sizes)
                top_indices = size_sorter[-top_n:]
                n_biggest_mask = np.zeros(len(sizes))
                n_biggest_mask[top_indices] = 1

            mask = np.logical_and(big_enough_mask, close_enough_mask)
            #mask = np.logical_and(n_biggest_mask, mask)
            mask = np.logical_not(mask)

            distances[mask] = 2*max_dist
        
        closest = np.argmin(distances)
        size = sizes[closest]
        
    elif n_labels == 1:
        closest = 1
        size = binary_im.sum()
    else:
        raise ValueError('n_labels < 0!')

    labels[labels!=closest] = 0
    labels[labels==closest] = 1
    return labels, size 



def xy_feature_images(width, height):
    """
    Returns two images of width and height. The first image has a value equal to the x coordinate at each pixel. The second image has a value equal to the y coordinate at each pixel.
    """
    xcoor = np.array([np.arange(width)] * height)
    ycoor = np.array([np.arange(height)] * width).transpose()
    return xcoor, ycoor


def xy_features(width, height):
    """
    Returns two vectorized images of width and height. The first image has a value equal to the x coordinate at each pixel. The second image has a value equal to the y coordinate at each pixel.
    """
    xcoor, ycoor = xy_feature_images(width, height)
    xcoor = xcoor.flatten()
    ycoor = ycoor.flatten()
    xcoor = np.reshape(xcoor, [xcoor.size,1])
    ycoor = np.reshape(ycoor, [ycoor.size,1])
    return xcoor, ycoor




class ImageFeatures(object):
    """ Converts an image into a set of features """

    def __init__(self, image, use_texture):
        """
        Create an ImageFeatures object for an input image. use_texture is a boolean that when true results in the inclusion of texture features in addition to spatial location and color features. 
        """
        self.image = image
        self.im_size = cv.cvGetSize(image)
        self.im_width = self.im_size.width
        self.im_height = self.im_size.height
        self.im_colors = 3
        self.array_image = ut.cv2np(image)
        self.tex_feat = None
        self.selected_features = None
        #self.im_width = image.shape[1]
        #self.im_height = image.shape[0]
        #self.im_colors = image.shape[2]
        self.use_texture = use_texture
        self.create_features(self.use_texture)
        self.mask_image = None

    def texture_features(self, block_size=5, filter_size=3):
        """
        Calculates the texture features associated with the image.
        block_size gives the size of the texture neighborhood to be processed
        filter_size gives the size of the Sobel operator used to find gradient information
        """
        #block_size = cv.cvSize(block_size, block_size)

        #convert to grayscale float
        channels = 1
        self.gray_image = cv.cvCreateImage(cv.cvSize(self.im_width, self.im_height),
                                           cv.IPL_DEPTH_8U, #cv.IPL_DEPTH_16U, #cv.IPL_DEPTH_32F,
                                           channels)


        #cv.CV_32FC1, #cv.IPL_DEPTH_32F, #cv.IPL_DEPTH_8U, #cv.IPL_DEPTH_16U, 
        channels = 1
        eig_tex = cv.cvCreateImage(cv.cvSize(self.im_width*6, self.im_height),
                                    cv.IPL_DEPTH_32F, 
                                    channels)


        cv.cvCvtColor(self.image, self.gray_image, cv.CV_BGR2GRAY);

        #cv.cvAdd(const CvArr* src1, const CvArr* src2, CvArr* dst, const CvArr* mask=NULL );
        
        #highgui.cvConvertImage(self.image, self.gray_image)
        
        cv.cvCornerEigenValsAndVecs(self.gray_image, eig_tex,#CvArr* eigenvv,
                                    block_size, filter_size)

        eig_tex = ut.cv2np(eig_tex)
        eig_tex = np.reshape(eig_tex, [self.im_height, self.im_width, 6])
        #print eig_tex.shape ## [480,640,3]
        ## (l1, l2, x1, y1, x2, y2), where
        ## l1, l2 - eigenvalues of M; not sorted
        ## (x1, y1) - eigenvector corresponding to l1
        ## (x2, y2) - eigenvector corresponding to l2
        tex_feat = np.zeros([3, self.im_height * self.im_width], dtype=np.float32)
        tmp = np.reshape(eig_tex, [self.im_height * self.im_width, 6]).T
        s = tmp[0] > tmp[1]
        tex_feat[1:3, s] = tmp[0, s] * tmp[2:4, s]
        tex_feat[0, s] = tmp[1, s]
        tex_feat[1:3, -s] = tmp[1, -s] * tmp[4:6, -s]
        tex_feat[0, -s] = tmp[0, -s]
        
        self.tex_feat = tex_feat.T
        self.tex_image = np.reshape(self.tex_feat, [self.im_height, self.im_width, 3])

        
    ## possible features to be included
    ##    + spatially smoothed features for neighborhood and multi-scale information
    ##    + orientation / gradient-based features
    def create_features(self, use_texture):
        """
        prepare the feature vectors from the image
        """

        self.xcoor, self.ycoor = xy_features(self.im_width, self.im_height)
        self.rgb = np.reshape(self.array_image, [self.im_height * self.im_width, self.im_colors])

        if use_texture and (self.tex_feat is None):
            self.texture_features()
            tex = self.tex_feat
            self.features = np.concatenate((self.xcoor,
                                            self.ycoor,
                                            self.rgb,
                                            self.tex_feat), axis=1).transpose()
        else:
            self.features = np.concatenate((self.xcoor,
                                            self.ycoor,
                                            self.rgb), axis=1).transpose()



    def select_subset(self, nsamples, mask_image=None):
        """
        sample a subset of the features
        mask_image should be a binary numpy array with dimensions equal to the image (True is use, False is don't use)
        """
        self.mask_image = mask_image
        if mask_image is not None:
            mask = np.reshape(mask_image, [self.im_height * self.im_width])
            masked_features = self.features[:, mask]
        else:
            masked_features = self.features

        dex = np.arange(masked_features.shape[1])
        select = rn.permutation(dex)[:nsamples]
        self.selected_features = masked_features[:,select]
        return self.selected_features

        

    
class SegmentObject(object):
    """ Segments an image into foreground and background assuming that a single object is in the center of the image """

    def __init__(self, image, features_object,
                 iter_limit=30,
                 object_center=None, object_diameter=None, mix_obj=None,
				 prior_gmm=None):
        """
        Create a SegmentObject given an image and a features_object computed from that image (ImageFeatures).
        """
        self.image = image
        self.im_size = cv.cvGetSize(image)
        self.im_width = self.im_size.width
        self.im_height = self.im_size.height
        
        if object_center is None:
            self.object_center = [self.im_width/2.0, self.im_height/2.0]
            
        if object_diameter is None:
            self.object_diameter = self.im_width/4.0
            self.object_diameter = self.im_width/6.0
            #self.object_diameter = self.im_width/8.0
            
        self.mix_obj = mix_obj
        if self.mix_obj is None:
            self.mix_obj = 1.0/25.0 #16.0  #20.0
        self.mix_bg = 1.0 - self.mix_obj

        #self.im_width = image.shape[1]
        #self.im_height = image.shape[0]
        #self.image_size = cv.cvSize(self.im_width, self.im_height)
        if features_object.selected_features is not None:
            self.features = features_object.selected_features
        else:
            self.features = features_object.features
        self.features_object = features_object
        self.iter_limit = iter_limit
        self.fit(prior_gmm)
        self.class_image = None
        self.clean_class_image = None
        self.large_obj = None
        self.fg_object_ellipse = None

    def fit(self, prior_gmm=None):
        """
        fit mixture of gaussians to the image features
        """
        points = np.matrix(self.features)
        num_gaussians = 2
        if True:
            self.n_feat = self.features.shape[0]
            if self.n_feat == 5:
                mean = np.matrix([self.object_center[0],
                                  self.object_center[1],
                                  128, 128, 128]).T
                svar = np.square(self.object_diameter/2.0)
                cvar = np.square(128.0)
                cov_obj = np.matrix([[svar,   0.0,   0.0,   0.0,   0.0],
                                     [ 0.0,  svar,   0.0,   0.0,   0.0],
                                     [ 0.0,   0.0,  cvar,   0.0,   0.0],
                                     [ 0.0,   0.0,   0.0,  cvar,   0.0],
                                     [ 0.0,   0.0,   0.0,   0.0,  cvar]])
                var = np.square(self.im_width/2.0)
                cov_bg = np.matrix([[ var,   0.0,   0.0,   0.0,   0.0],
                                    [ 0.0,   var,   0.0,   0.0,   0.0],
                                    [ 0.0,   0.0,  cvar,   0.0,   0.0],
                                    [ 0.0,   0.0,   0.0,  cvar,   0.0],
                                    [ 0.0,   0.0,   0.0,   0.0,  cvar]])
            elif self.n_feat == 8:
                mean = np.matrix([self.object_center[0],
                                  self.object_center[1],
                                  128, 128, 128,
                                  0.0, 0.0, 0.0]).T
                svar = np.square(self.object_diameter/2.0)
                cvar = np.square(128.0)
                cov_obj = np.matrix([[svar,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
                                     [ 0.0,  svar,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
                                     [ 0.0,   0.0,  cvar,   0.0,   0.0,   0.0,   0.0,   0.0],
                                     [ 0.0,   0.0,   0.0,  cvar,   0.0,   0.0,   0.0,   0.0],
                                     [ 0.0,   0.0,   0.0,   0.0,  cvar,   0.0,   0.0,   0.0],
                                     [ 0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   0.0],
                                     [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0],
                                     [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0]])
                                     
                var = np.square(self.im_width/2.0)
                
                cov_bg = np.matrix([[ var,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
                                    [ 0.0,   var,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
                                    [ 0.0,   0.0,   var,   0.0,   0.0,   0.0,   0.0,   0.0],
                                    [ 0.0,   0.0,   0.0,  cvar,   0.0,   0.0,   0.0,   0.0],
                                    [ 0.0,   0.0,   0.0,   0.0,  cvar,   0.0,   0.0,   0.0],
                                    [ 0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   0.0],
                                    [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0,   0.0],
                                    [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   1.0]])
            else:
                raise AssertionError('unrecognized number of features (%d) given to fit (not 5 or 8)' % self.n_feat)

            g_obj = pb.Gaussian(mean, cov_obj)
            g_bg = pb.Gaussian(mean, cov_bg)
            start_gaussians = [(g_obj, self.mix_obj), (g_bg, self.mix_bg)]

            if prior_gmm is not None:
                self.gmm = gm.GMM(points=points, start_gaussians=prior_gmm.gaussians)
            else:
                for i,c in enumerate(start_gaussians):
                    print 'start_gaussians[%d] :' % i
                    print c[0]
                    print c[1]

                self.gmm = gm.GMM(points=points,
                                  start_gaussians=start_gaussians)
                self.gmm.fit(iter_limit=self.iter_limit)
            
        else:
            self.gmm = gm.GMM(points=points,
                              start_gaussians=gm.rand_start(num_gaussians, points))
            self.gmm.fit(iter_limit=self.iter_limit)


    def classify_image(self, use_flip_heuristic=True):
        """
        classify the input image using the GMM
        results in a binary image
        """
        
        channels = 3
        member = self.gmm.membership_mat()
        all_feats = self.features_object.features
        membership = member(all_feats)
        gindex = np.argmax(membership, axis=0)
        obj = gindex == 0
        obj_im = obj.astype(np.uint8) * 255
        obj_im = np.reshape(obj_im, [self.im_height, self.im_width])

        # find the category with the smallest spatial variance
        # minimum maximum eigenvalue
        
        # avoid category reversal (happened to me once, gaussians flipped roles in terms of object and background)
        if use_flip_heuristic:
            feature_mask = self.features_object.mask_image
            if feature_mask is not None:
                max_total = feature_mask.sum()
                category_total = (obj_im & feature_mask).sum()
            else:
        	    max_total = self.im_height * self.im_width
        	    category_total = obj_im.sum()
            if (category_total > (max_total - category_total)):
                obj_im = ~obj_im
        else:
            print "textseg.classify_image: not using flip heuristic!"

        #display_images([ut.np2cv(obj_im.astype(np.uint8)*255)])
        
        #class_image = ut.np2cv(obj_im)            
        self.class_image = obj_im #class_image


    def clean_classified_image(self):
        """
        clean the binary image resulting from pixel classification using morphological operators 
        """
        if self.class_image is None:
            self.classify_image()

        bim = self.class_image
        feature_mask = self.features_object.mask_image
        if feature_mask is not None:
            bim = bim & feature_mask

        bim = ni.binary_fill_holes(bim)
        min_gap = 0
        for n in range(min_gap):
            bim = ni.binary_dilation(bim)
            #bim = ni.binary_closing(bim)
        #bim = ni.binary_fill_holes(bim)
        min_radius = 8
        for n in range(min_radius):
            bim = ni.binary_erosion(bim)
            #bim = ni.binary_opening(bim)
        for n in range(min_radius):
            bim = ni.binary_dilation(bim)
        #bim = ni.binary_dilation(bim)
        #bim = ni.binary_erosion(bim)
        self.clean_class_image = bim.astype(np.uint8) * 255


    def find_largest_object(self):
        """
        find the largest object in the classified binary image using connected components
        """
        if self.clean_class_image is None:
            self.clean_classified_image()
        obj, obj_size = largest_region(self.clean_class_image)
        self.large_obj = obj.astype(np.uint8) * 255


    def find_best_object(self):
        """
        find the an object in the classified binary image using connected components that is both large and close to the center
        """
        if self.clean_class_image is None:
            self.clean_classified_image()
            
        obj, obj_size = big_center_region(self.clean_class_image)
        self.large_obj = obj.astype(np.uint8) * 255


    def fit_to_largest_object(self):
        """
        fit an ellipse to the segmented, cleaned, and selected object
        """
        if self.large_obj is None:
            self.find_largest_object()

        obj_mask = self.large_obj > 0
        h, w = obj_mask.shape
        xcoor, ycoor = xy_features(w,h)
        obj_mask = obj_mask.flatten()
        xcoor = xcoor[obj_mask]
        ycoor = ycoor[obj_mask]
        points = np.concatenate((xcoor, ycoor), axis=1).transpose()
        points = np.matrix(points)

        print "points shape:"
        print points.shape
        
        start = gm.simple_start(1, points)
        gmm = gm.GMM(points, start)
        gmm.fit(iter_limit=self.iter_limit)
        
        #weights = np.matrix(np.ones(points.shape[0]))
        #gauss = pb.Gaussian(0,0,dimension=2)

        self.fg_object_ellipse = gmm2ellipses(gmm)[0] 

        if False:
            tmp = points.astype(np.float32).T
            tmp = np.reshape(tmp, [1, tmp.shape[1], 2])
            cvpoints = ut.np2cv(tmp)
            ellipse = cv.cvFitEllipse2(cvpoints)
            self.fg_object_ellipse = ut.Ellipse()
            self.fg_object_ellipse.set_from_cvbox2d(ellipse)


            #typedef struct CvBox2D
            #{
            #    CvPoint2D32f center;  /* center of the box */
            #    CvSize2D32f  size;    /* box width and length */
            #    float angle;          /* angle between the horizontal axis
            #    and the first side (i.e. length) in radians */
            #    }
            #CvBox2D;

            #The function cvFitEllipse calculates ellipse that fits best (in least-squares sense) to a set of 2D points. The meaning of the returned structure fields is similar to those in cvEllipse except that size stores the full lengths of the ellipse axises, not half-lengths

            #center
            #Center of the ellipse. 
            #axes
            #Length of the ellipse axes. 
            #angle
            #Rotation angle.
        

    def get_images_for_display(self, draw_mixture=True):
        """
        returns a list of images that show the intermediate and final results of the image segmentation
        """

        image_list = []

        ## display the resulting gaussians
        if draw_mixture:
            for i, ellipse in enumerate(gmm2ellipses(self.gmm)):
                ellipse.draw_on_image(self.image,
                                      color=(255*i,255,255*i),
                                      principal_axis_color=(255,255,255))
                
            if self.fg_object_ellipse is not None:
                self.fg_object_ellipse.draw_on_image(self.image,
                                                     color=(255, 0, 0),
                                                     principal_axis_color=(255,255,255))

        image_list.append(self.image)
        
        if self.class_image is not None:
            if str(type(self.class_image)).find('opencv') == -1:
                disp_im = ut.np2cv(self.class_image)
            else:
                disp_im = self.class_image
            image_list.append(disp_im)

        if self.clean_class_image is not None:
            if str(type(self.clean_class_image)).find('opencv') == -1:
                clean_disp_im = ut.np2cv(self.clean_class_image)
            else:
                clean_disp_im = self.class_image
            image_list.append(clean_disp_im)
            
        if self.large_obj is not None:
            if str(type(self.large_obj)).find('opencv') == -1:
                obj_disp_im = ut.np2cv(self.large_obj)
            else:
                obj_disp_im = self.class_image
            if draw_mixture:
                if self.fg_object_ellipse is not None:
                    self.fg_object_ellipse.draw_on_image(obj_disp_im,
                                                         color=(255, 0, 0),
                                                         principal_axis_color=(255,255,255))
            image_list.append(obj_disp_im)
                
        return image_list



def segment_center_object(image,
                          display_on=False,
                          nsamp=10000, iter_limit=30,
                          use_texture=True, use_hsv=True, set_v_to_zero=True,
                          use_mask=True, remove_saturation=False, remove_boundary = True, prior_gmm=None, 
						  use_flip_heuristic=True):
    """
    segment the input image (OpenCV image) and return an ellipse fit to the center object (foreground) and a binary image mask for this foreground object
    
    nsamp : number of pixels to be used when fitting the Gaussian mixture model (impacts speed and accuracy)
    iter_limit : maximum number of iterations when fitting the texture model
    use_texture : use texture features
    use_hsv : use hsv color space for features
    set_v_to_zero : effectively remove the value (brightness) component of the hsv features
    use_mask : mask out areas of the image prior to training the appearance model and segmenting
    remove_saturation : if use_mask, then remove saturated pixels (RGB values = 255 = max value)
    remove_boundary : if use_mask, then remove the borders of the image prior to segmenting it

    returns a segmentation object (SegmentObject)

    """
    
    #remove_low_freq = True

    if use_hsv:
        hsv_image = cv.cvCreateImage(cv.cvSize(image.width, image.height),
                                     cv.IPL_DEPTH_8U, 3)
        cv.cvCvtColor(image, hsv_image, cv.CV_RGB2HSV) #cv.CV_BGR2HSV)
        if set_v_to_zero:
            #cvSet(hsv_image, cvScalarAll(0), )
            for y in xrange(hsv_image.height):
                for x in xrange(hsv_image.width):
                    pix = hsv_image[y,x]
                    hsv_image[y,x] = cv.cvScalar(pix[0], pix[1], 0.0)
        image = hsv_image


    if display_on: 
        image_list = []
    else:
        image_list = None
        
    imf = ImageFeatures(image, use_texture=use_texture)
    #imf.texture_features()
    if use_mask:
        #test_mask = np.zeros([image.height, image.width])
        #test_mask[0:200, 0:200] = 1.0
        #test_mask = test_mask > 0.0
        # select saturation mask
        nim = ut.cv2np(image)

        if remove_saturation:
            # remove saturated pixels
            #saturation_mask = ~np.alltrue(nim > 255, axis=2)
            saturation_mask = ~np.any(nim >= 255, axis=2)
            #saturation_mask = np.sum(nim >= 255, axis=2) < 2

        if remove_boundary:
            # remove boundaries beyond the possible object size
            border_y = 50
            border_x = 100
            too_big_mask = np.zeros(nim.shape[:2], dtype=np.bool)
            w = nim.shape[1]
            h = nim.shape[0]
            too_big_mask[border_y : h - border_y, border_x : w - border_x] = True

        if remove_saturation and remove_boundary:
            feature_mask = saturation_mask & too_big_mask
        elif remove_saturation:
            feature_mask = saturation_mask
        else:
            feature_mask = too_big_mask
        disp_mask = feature_mask.copy()

        features = imf.select_subset(nsamp, mask_image=feature_mask)
        cv_mask = ut.np2cv(disp_mask.astype(np.uint8) * 255)
        if image_list is not None:
            image_list.append(cv_mask)
    else:
        features = imf.select_subset(nsamp)
    #sego = SegmentObject(image, features, iter_limit=iter_limit)
    sego = SegmentObject(image, imf, iter_limit=iter_limit, prior_gmm=prior_gmm)
    sego.classify_image(use_flip_heuristic)
    sego.clean_classified_image()
    #sego.find_largest_object()
    sego.find_best_object()
    sego.fit_to_largest_object()
    if image_list is not None:
        image_list.extend(sego.get_images_for_display())
        ut.display_images(image_list)
    return sego



def add_options(parser):
    """
    adds options used when texseg is run from the command line
    """
    parser.add_option('--load', action='store', type='string', dest='load',
                      default=None,
                      help='load an image for processing')
    return parser


if __name__ == '__main__':
    """
    Example Code
    """
    
    parser = OptionParser()
    parser = add_options(parser)
    (options, args) = parser.parse_args()

    if options.load is not None:
        image_filename = options.load
    else:
        image_filename = './texseg_example_image.png'
#        image_filename = 'test_cv.bmp'

    image = highgui.cvLoadImage(image_filename)
    sego = segment_center_object(image, display_on=True)
    
    print '********************************************************'
    print
    print 'this is the ellipse for the segmented foreground object:'
    print
    print sego.fg_object_ellipse
    print
    print '********************************************************'
        


####
# notes for development:
#
#  1. use motion via contact
#  2. use laser range finder readings visible through the eye in hand camera
#  *done* 3. filter out saturated regions prior to texture segmentation
#  4. move the camera and use motion/stereo to filter out visual features that are below the plane (this should help with reflections and transparency
#  5. use multiple gaussian models (i.e., more than 2, for example 2 for the background)
#  6. use the laser to set the initial guess size and position of the object cluster
#  7. use collected information about surfaces from base camera and during room exploration
#
# other desires:
#  *done* 1. faster fitting (see if matrix operations could help)!!!
#  *done* 2. most centered instead of most pixels?
####
    




