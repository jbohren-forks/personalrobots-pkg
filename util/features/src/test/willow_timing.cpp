#include "detector.h"
#include "timer.h"
#include <cv.h>
#include <highgui.h>

int main( int argc, char** argv )
{
    assert(argc > 1);
    
    IplImage* source = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    WgDetector detector(cvSize(source->width, source->height));
    std::vector<Keypoint> keypts;

    keypts = detector.DetectPoints(source);

    {
        Timer t("Willow detector (20x)");
        for (int i = 0; i < 20; ++i) {
            keypts = detector.DetectPoints(source);
        }
    }

    return 0;
}
