#include "calonder_descriptor/matcher.h"
#include "calonder_descriptor/rtree_classifier.h"
#include "detectors.h"
#include "transform_utils.h"
#include <cvwimage.h>
#include <highgui.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cassert>
#include <fstream>
#include <cstdlib>
#include <cstdio>

namespace po = boost::program_options;
using namespace features;
using std::string;

void writeSignature(std::ostream &os, const float* sig, size_t size)
{
  std::copy(sig, sig + size, std::ostream_iterator<float>(os, " "));
  os << std::endl;
}

int main( int argc, char** argv )
{
  int num_keypts;
  unsigned long seed = std::time(NULL);
  float threshold;
  string trees_file, source_file, test_file, transform_file;
  string patch_dir;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("classifier,c", po::value<string>(), "forest classifier file")
    ("source,s", po::value<string>(), "source (detection) image")
    ("test,t", po::value<string>(), "test (matching) image")
    ("transform,x", po::value<string>(), "source->test transform file")
    ("keypoints,k", po::value<int>(&num_keypts)->default_value(300),
     "number of keypoints")
    ("threshold", po::value<float>(&threshold)->default_value(0.005),
     "sparse signature threshold")
    ("source-sigs", po::value<string>(), "save signatures from source image")
    ("test-sigs", po::value<string>(), "save signatures from test image")
    ("patches", po::value<string>(), "save patches to directory")
    ("sift", "use SIFT detector (default is star)")
    ("seed", po::value<unsigned long>(&seed), "set PRNG seed");

  po::positional_options_description pos;
  pos.add("classifier", 1);
  pos.add("source", 1);
  pos.add("test", 1);
  pos.add("transform", 1);
  
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).
            positional(pos).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  if (!vm.count("classifier")) {
    std::cout << "Error: must specify forest classifier file"
              << std::endl << desc << std::endl;
    return 1;
  }
  trees_file = vm["classifier"].as<string>();
  
  if (!vm.count("test")) {
    std::cout << "Error: must specify test image" << std::endl
              << desc << std::endl;
    return 1;
  }
  test_file = vm["test"].as<string>();

  if (vm.count("source")) {
    source_file = vm["source"].as<string>();
  } else {
    int index = test_file.find_last_of('.');
    source_file = test_file;
    source_file[index-1] = '1';
  }

  if (vm.count("transform")) {
    transform_file = vm["transform"].as<string>();
  } else {
    int index = test_file.find_last_of('.');
    transform_file = test_file.substr(0, index - 4) + "H1to" + test_file[index-1] + 'p';
  }

  std::ofstream src_sig_file, test_sig_file;
  bool save_src_sigs = vm.count("source-sigs");
  bool save_test_sigs = vm.count("test-sigs");
  if (save_src_sigs)
    src_sig_file.open( vm["source-sigs"].as<string>().c_str() );
  if (save_test_sigs)
    test_sig_file.open( vm["test-sigs"].as<string>().c_str() );

  std::ofstream matches_file;
  bool save_patches = vm.count("patches");
  if (save_patches) {
    patch_dir = vm["patches"].as<string>();
    string file_name = patch_dir + "/matches.txt";
    matches_file.open( file_name.c_str() );
  }
  
  RTreeClassifier classifier;
  classifier.read(trees_file.c_str());
  classifier.setThreshold(threshold);
  Rng rng(seed);
  
  cv::WImageBuffer1_b src_img( cvLoadImage(source_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE) );
  cv::WImageBuffer1_b test_img( cvLoadImage(test_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE) );
  CvMat *transform = cvCreateMat(3, 3, CV_32FC1);
  ReadTransform(transform_file.c_str(), transform);
  
  // Detect points in source image
  std::vector<Keypoint> keypts;
  if (vm.count("sift"))
    keypts = siftKeypoints(src_img.Ipl());
  else
    keypts = starKeypoints(src_img.Ipl(), 7, 0);
  keypts.erase(std::remove_if(keypts.begin(), keypts.end(),
                              OutsideSource(test_img.Width(), test_img.Height(), transform)),
               keypts.end());
  std::sort(keypts.begin(), keypts.end());
  assert((int)keypts.size() >= num_keypts);
  keypts.erase(keypts.begin() + num_keypts, keypts.end());

  size_t sig_size = classifier.classes();
  BruteForceMatcher<CvPoint> matcher(sig_size);

  // Extract patches and add their signatures to matcher database
  int index = 0;
  float* sig_buffer = NULL;
  posix_memalign(reinterpret_cast<void**>(&sig_buffer), 16, sig_size * sizeof(float) * keypts.size());
  float* sig = sig_buffer;
  BOOST_FOREACH( Keypoint &pt, keypts ) {
    cv::WImageView1_b view = extractPatch(src_img.Ipl(), pt);
    classifier.getSignature(view.Ipl(), sig);
    matcher.addSignature(sig, cvPoint(pt.x, pt.y));

    if (save_src_sigs)
      writeSignature(src_sig_file, sig, sig_size);

    if (save_patches) {
      char file_name[128];
      sprintf(file_name, "%s/source%i.pgm", patch_dir.c_str(), index);
      cvSaveImage(file_name, view.Ipl());
      ++index;
    }
    sig += sig_size;
  }

  float d1, d2;
  int correct = 0, second = -1;
  index = 0;
  CvRect window = cvRect(32, 32, 224, 224);
  posix_memalign(reinterpret_cast<void**>(&sig), 16, sig_size * sizeof(float));
  BOOST_FOREACH( Keypoint &pt, keypts ) {
    CvPoint warped_pt = MapPoint(cvPoint(pt.x, pt.y), transform);
    cv::WImageView1_b view = extractPatch(test_img.Ipl(), warped_pt);
    classifier.getSignature(view.Ipl(), sig);
    int match = matcher.findMatches(sig, &d1, &second, &d2);
    //int match = matcher.findMatchInWindow(sig, window, &d1);

    printf("%i -> %i (%i), ", index, match, second);
    if (match == index) {
      ++correct;
      printf("correct\n");
    } else {
      printf("wrong\n");
    }
    if (match != -1) {
      CvPoint match_pt = matcher.getData(match);
      printf("\t(x, y) = (%i, %i)\n", match_pt.x, match_pt.y);
    }
    printf("\td1 = %f, d2 = %f\n\td2/d1 = %f\n", d1, d2, d2/d1);

    if (save_patches) {
      char file_name[128];
      sprintf(file_name, "%s/test%i.pgm", patch_dir.c_str(), index);
      cvSaveImage(file_name, view.Ipl());

      matches_file << index << ' ' << match << std::endl;
    }

    if (save_test_sigs)
      writeSignature(test_sig_file, sig, sig_size);
    
    ++index;
  }
  free(sig);

  printf("\nCorrect: %i / %i = %f%%\n\n", correct, num_keypts, 100.0f*correct/num_keypts);

  cvReleaseMat(&transform);
  free(sig_buffer);
  if (save_src_sigs) src_sig_file.close();
  if (save_test_sigs) test_sig_file.close();
  if (save_patches) matches_file.close();

  return 0;
}
