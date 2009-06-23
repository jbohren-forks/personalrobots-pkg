#include "calonder_descriptor/randomized_tree.h"
#include <highgui.h>
#include <cvwimage.h>
#include <cstdio>
#include <ctime>

using namespace features;

int main( int argc, char** argv )
{
  cv::WImageBuffer1_b source( cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE) );
  std::vector<BaseKeypoint> base_set;
  base_set.push_back( BaseKeypoint(467, 264, source.Ipl()) );
  base_set.push_back( BaseKeypoint(333, 191, source.Ipl()) );
  base_set.push_back( BaseKeypoint(494, 490, source.Ipl()) );
  base_set.push_back( BaseKeypoint(268, 184, source.Ipl()) );
  base_set.push_back( BaseKeypoint(90, 580, source.Ipl()) );
  
  RandomizedTree rtree;
  Rng rng( std::time(NULL) );
  rtree.train(base_set, rng);
  //rtree.read(argv[2]);

  cvSmooth(source.Ipl(), source.Ipl());

  int size = RandomizedTree::PATCH_SIZE;
  std::vector<BaseKeypoint>::const_iterator key_it;
  for (key_it = base_set.begin(); key_it != base_set.end(); ++key_it) {
    cv::WImageView1_b patch(&source, key_it->x - size/2, key_it->y - size/2, size, size);
    float *post = rtree.getPosterior( patch.Ipl() );

    float max_prob = 0.2;
    int best_class = -1;
    for (int i = 0; i < (int)base_set.size(); ++i) {
      float prob = post[i];
      printf("%f ", prob);
      if (prob > max_prob) {
        max_prob = prob;
        best_class = i;
      }
    }
    printf("; class = %i\n", best_class);
  }
  
  if (argc > 2)
    rtree.write(argv[2]);
  
  return 0;
}
