#ifndef __IMAGE_CLASSIFIER_H__
#define __IMAGE_CLASSIFIER_H__

/**
   @brief Abstract base class for image classifiers
 */
class ImageClassifier {
public:
  virtual ~ImageClassifier() { };

  /**
     @brief Loads training data
     @param trainImages Training images
     @param trainSeg Training segmentations
   */
  virtual void loadTrainingData(const vector<IplImage*>& trainImages,
				const vector<IplImage*>& trainSeg) = 0;

  /**
     @brief Trains classifier for a certain number of iterations
     @param iterations Number of iterations for which to train
   */
  virtual void train(int iterations) = 0;

  /**
     @brief Evaluate the classifer on the given image
     @param image Image to be evaluated
     @param segmented Pre-allocated output argument for the segmented 
     image
   */
  virtual void evaluate(const IplImage* image, IplImage* segmented) = 0;
};

#endif
