/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Marius Muja

#ifndef CHAMFER_MATCHING_H_
#define CHAMFER_MATCHING_H_

#include <vector>

using namespace std;

typedef pair<int,int> coordinate_t;
typedef float orientation_t;;
typedef vector<coordinate_t> template_coords_t;
typedef vector<orientation_t> template_orientations_t;



/**
 * Finds a contour in an edge image. The original image is altered by removing the found contour.
 * @param templ_img Edge image
 * @param coords Coordinates forming the contour.
 * @return True while a contour is still found in the image.
 */
bool findContour(IplImage* templ_img, template_coords_t& coords);


/**
* Computes contour points orientations using the approach from:
*
* Matas, Shao and Kittler - Estimation of Curvature and Tangent Direction by
* Median Filtered Differencing
*
* @param coords Contour points
* @param orientations Contour points orientations
*/
void findContourOrientations(const template_coords_t& coords, template_orientations_t& orientations);




/**
 * Class that represents a template for chamfer matching.
 */
class ChamferTemplate
{
public:
	template_coords_t coords;
	template_orientations_t orientations;
	CvSize size;

	ChamferTemplate()
	{

	}

	ChamferTemplate(IplImage* edge_image);


	/**
	 * Resizes a template
	 *
	 * @param scale Scale to be resized to
	 */
	void rescale(float scale);

	void show();
};


/**
 * Used to represent a matching result.
 */
class ChamferMatch
{
public:
	CvPoint offset;
	float distance;
	const ChamferTemplate* tpl;
};



/**
 * Implements the chamfer matching algorithm on images taking into account both distance from
 * the template pixels to the nearest pixels and orientation alignment between template and image
 * contours.
 */
class ChamferMatching
{
	float min_scale;
	float max_scale;
	int count_scale;

	float truncate;

	vector<ChamferTemplate*> templates;

public:
	ChamferMatching() : min_scale(0.5), max_scale(1.5), count_scale(5), truncate(20)
	{

	}

	~ChamferMatching()
	{
		for (size_t i = 0; i<templates.size(); i++) {
			delete templates[i];
		}
	}

	/**
	 * Add a template to the detector from an edge image.
	 * @param templ An edge image
	 */
	void addTemplateFromImage(IplImage* templ);

	/**
	 * Run matching usin an edge image.
	 * @param edge_img Edge image
	 * @return a match object
	 */
	ChamferMatch matchEdgeImage(IplImage* edge_img);

	/**
	 * Run matching using a regular image.
	 *
	 * Will run Canny edge detection and then the edge matching.
	 * @param img
	 * @param high_threshold The high edge threshold to use in Canny edge detector.
	 * @param low_threshold The low edge threshold to use in Canny edge detector. If default (-1) is used it's computed as high_threshold/2.
	 * @return a match object
	 */
	ChamferMatch matchImage(IplImage* img, int high_threshold = 160, int low_threshold = -1);

private:




	/**
	 * Computes annotated distance transform.
	 *
	 * @param edges_img Edge image.
	 * @param dist_img Distance image, each pixel represents the distance to the nearest edge.
	 * @param annotate_img Two channel int image, each 'pixel' represents the coordinate of the nearest edge.
	 * @param orientation_img Orientation image, should contain the orientations of each edge pixel. Only pixels with
	 * valid orientations are processed in the distance transform.
	 * @param truncate Value to truncate the distance transform to, no value bigger than this is allowed in the output.
	 * This helps when the contours of objects are interrupted, so we don't penalize the gaps too much.
	 * @param a Horizontal distance.
	 * @param b Diagonal distance.
	 */
	void computeDistanceTransform(IplImage* edges_img, IplImage* dist_img, IplImage* annotate_img, IplImage* orientation_img, float truncate = -1, float a = 1.0, float b = 1.5 );


    /**
     * Computes teh orientations of edges in an edge image,
     * @param edge_img Edge image
     * @param orientation_img Image of same size as edge image that will be filed in with the orientations of each edge pixel in the edge image.
     */
    void computeEdgeOrientations(IplImage* edge_img, IplImage* orientation_img);


    /**
     * For each non-edge pixels sets it's orientation the same with the closest edge pixel.
     * @param annotated_img
     * @param orientation_img
     */
    void fillNonContourOrientations(IplImage* annotated_img, IplImage* orientation_img);


    /**
     * Computes the chamfer matching cost for one position in the target image.
     * @param dist_img Distance transform image.
     * @param orientation_img Orientation image.
     * @param templ_addr Offsets of the template points into the target image (used to speedup the search).
     * @param templ_orientations Orientations of the target points.
     * @param offset Offset where to compute cost
     * @param alpha Weighting between distance cost and orientation cost.
     * @return Chamfer matching cost
     */
    float localChamferDistance(IplImage* dist_img, IplImage* orientation_img, const vector<int>& templ_addr, const vector<float>& templ_orientations, CvPoint offset, float alpha = 0.7);


    /**
     * Matches a template throughout an image.
     * @param dist_img Distance transform image.
     * @param orientation_img Orientation image.
     * @param tpl Template to match
     * @param cm Matching result
     */
    void matchTemplate(IplImage* dist_img, IplImage* orientation_img, const ChamferTemplate& tpl, ChamferMatch& cm);


    /**
     * Matches all templates.
     * @param dist_img Distance transform image.
     * @param orientation_img Orientation image.
     * @param cm Matching result
     */
    void matchTemplates(IplImage* dist_img, IplImage* orientation_img, ChamferMatch& cm);



};








#endif /* CHAMFER_MATCHING_H_ */
