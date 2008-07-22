from opencv import cv
import numpy as np

cv2np_type_dict = {cv.CV_16S	  : (np.int16, 1),	
				   cv.CV_16SC	 : (np.int16, 1),   
				   cv.CV_16SC1	: (np.int16, 1),   
				   cv.CV_16SC2	: (np.int16, 2),   
				   cv.CV_16SC3	: (np.int16, 3),   
				   cv.CV_16SC4	: (np.int16, 4),   
				   cv.CV_16U	  : (np.uint16, 1),   
				   cv.CV_16UC	 : (np.uint16, 1),   
				   cv.CV_16UC1	: (np.uint16, 1),   
				   cv.CV_16UC2	: (np.uint16, 2),   
				   cv.CV_16UC3	: (np.uint16, 3),   
				   cv.CV_16UC4	: (np.uint16, 4),   
				   cv.CV_32F	  : (np.float32, 1),   
				   cv.CV_32FC	 : (np.float32, 1),   
				   cv.CV_32FC1	: (np.float32, 1),   
				   cv.CV_32FC2	: (np.float32, 2),   
				   cv.CV_32FC3	: (np.float32, 3),   
				   cv.CV_32FC4	: (np.float32, 4),   
				   cv.CV_32S	  : (np.int32, 1),   
				   cv.CV_32SC	 : (np.int32, 1),   
				   cv.CV_32SC1	: (np.int32, 1),   
				   cv.CV_32SC2	: (np.int32, 2),   
				   cv.CV_32SC3	: (np.int32, 3),   
				   cv.CV_32SC4	: (np.int32, 4),   
				   cv.CV_64F	  : (np.float64, 1),   
				   cv.CV_64FC	 : (np.float64, 1),   
				   cv.CV_64FC1	: (np.float64, 1),   
				   cv.CV_64FC2	: (np.float64, 2),   
				   cv.CV_64FC3	: (np.float64, 3),   
				   cv.CV_64FC4	: (np.float64, 4),   
				   cv.CV_8S	   : (np.int8, 1),   
				   cv.CV_8SC	  : (np.int8, 1),   
				   cv.CV_8SC1	 : (np.int8, 1),   
				   cv.CV_8SC2	 : (np.int8, 2),   
				   cv.CV_8SC3	 : (np.int8, 3),   
				   cv.CV_8SC4	 : (np.int8, 4),   
				   cv.CV_8U	   : (np.uint8, 1),   
				   cv.CV_8UC	  : (np.uint8, 1),   
				   cv.CV_8UC1	 : (np.uint8, 1),   
				   cv.CV_8UC2	 : (np.uint8, 2),   
				   cv.CV_8UC3	 : (np.uint8, 3),   
				   cv.CV_8UC4	 : (np.uint8, 4)}

def numpymat2cvmat(nmat):
    cvmat = cv.cvCreateMat(nmat.shape[0],nmat.shape[1],cv.CV_32FC1)
    for i in range(nmat.shape[0]):
        for j in range(nmat.shape[1]):
            #print cvmat[i][j]
            #print nmat[i,j]	  
            cvmat[i,j] = nmat[i,j]	  
    return cvmat

def cvmat2numpymat(cvmat):
	nmat = np.zeros((cvmat.width,cvmat.height))
	for i in range(cvmat.width):
		for j in range(cvmat.height):
			nmat[i][j] = cvmat[i][j]
	return nmat

def cv2np(im, format='RGB'):
	if format == 'BGR':
		cv.cvCvtColor( im, im, cv.CV_BGR2RGB )
	numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(im)]
	array_size = [im.height, im.width, nchannels]
	np_im = np.frombuffer(im.imageData, dtype=numpy_type, 
            count=im.height*im.width*nchannels*(im.depth/8))
	return np.reshape(np_im, array_size)

def list_mat_to_mat(list_mat, axis=0):
	return np.concatenate(tuple(list_mat), axis=axis)


