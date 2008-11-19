#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/rng.h"
#include "detectors.h"
#include "timer.h"
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <highgui.h>
#include <cvwimage.h>
#include <cstdio>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <string>
#include <fstream>
#include <map>
#include <cmath>
#include <cstdlib>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace features;
using std::string;

std::vector<BaseKeypoint>
selectBaseSet(std::vector<BaseKeypoint> &candidate_set,
              unsigned int N, Rng &rng)
{
  static const int radius_sq = 25;
  typedef std::vector<BaseKeypoint>::const_iterator iter;
  
  assert((int)candidate_set.size() >= classes);
  std::random_shuffle(candidate_set.begin(), candidate_set.end(), rng);
  
  // Nearness constraint
  std::vector<BaseKeypoint> base_set;
  base_set.reserve(N);
  iter cand_it = candidate_set.begin();
  while (base_set.size() < N && cand_it != candidate_set.end()) {
    iter base_it;
    for (base_it = base_set.begin(); base_it != base_set.end(); ++base_it) {
      if (base_it->image != cand_it->image)
        continue;
      int x_diff = base_it->x - cand_it->x;
      int y_diff = base_it->y - cand_it->y;
      if (x_diff*x_diff + y_diff*y_diff <= radius_sq)
        break;
    }

    if (base_it == base_set.end())
      base_set.push_back(*cand_it);
    
    ++cand_it;
  }
  
  return base_set;
}

int main( int argc, char** argv )
{
  int classes = 300, reduced_classes = 0, samples = 200;
  int trees = 0, depth = 0, views = 0;
  unsigned long seed = std::time(NULL);
  float threshold = 0;
  string patch_dir;
  double theta_min, theta_max, phi_min, phi_max, lambda_min, lambda_max;

  po::options_description generic_options("Generic options");
  generic_options.add_options()
    ("help,h", "produce help message")
    ("config", po::value<string>()->default_value("baseset.cfg"),
     "Use options from configuration file");
  
  po::options_description tree_options("Tree options");
  tree_options.add_options()
    ("trees,t", po::value<int>(&trees)->default_value(20), "number of trees")
    ("depth,d", po::value<int>(&depth)->default_value(12), "tree depth")
    ("classes,c", po::value<int>(&classes)->default_value(300), "number of classes")
    ("reduced,r", po::value<int>(&reduced_classes), "reduced number of classes");

  po::options_description view_options("View options");
  view_options.add_options()
    ("views,v", po::value<int>(&views)->default_value(1000),
     "number of random views of each base keypoint")
    ("noise,n", "add white noise during training")
    ("theta-min", po::value<double>(&theta_min)->default_value(-M_PI),
     "minimum viewpoint rotation")
    ("theta-max", po::value<double>(&theta_max)->default_value(M_PI),
     "maximum viewpoint rotation")
    ("phi-min", po::value<double>(&phi_min)->default_value(-M_PI),
     "minimum viewpoint skew rotation")
    ("phi-max", po::value<double>(&phi_max)->default_value(M_PI),
     "maximum viewpoint skew rotation")
    ("lambda-min", po::value<double>(&lambda_min)->default_value(0.6),
     "minimum viewpoint scaling")
    ("lambda-max", po::value<double>(&lambda_max)->default_value(1.5),
     "maximum viewpoint scaling");

  po::options_description file_options("File options");
  file_options.add_options()
    ("save-trees", po::value<string>(), "save trees to file")
    ("load-trees", po::value<string>(), "load trees from file")
    ("save-base-set", po::value<string>(), "save base set to file")
    //("load-base-set", po::value<string>(), "load base set from file")
    ("save-sigs", po::value<string>(), "save dense signatures to file")
    ("patches", po::value<string>(), "save patches to directory");

  po::options_description config_options("Configuration");
  config_options.add_options()
    ("samples", po::value<int>(&samples)->default_value(200),
     "number of keypoints sampled from each source image")
    ("thresh", po::value<float>(&threshold)->default_value(0), "signature threshold")
    ("sift", "use SIFT for detection")
    ("seed", po::value<unsigned long>(&seed), "set PRNG seed");

  po::options_description hidden_options("Hidden options");
  hidden_options.add_options()
    ("source", po::value< std::vector<string> >(), "source image file");
  
  po::positional_options_description pos;
  pos.add("source", -1);

  po::options_description cmdline_options;
  cmdline_options.add(generic_options).add(tree_options).add(view_options);
  cmdline_options.add(file_options).add(config_options).add(hidden_options);

  po::options_description config_file_options;
  config_file_options.add(tree_options).add(view_options);
  config_file_options.add(file_options).add(config_options).add(hidden_options);

  po::options_description visible_options("Allowed options");
  visible_options.add(generic_options).add(tree_options).add(view_options);
  visible_options.add(file_options).add(config_options);
  
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(cmdline_options).
            positional(pos).run(), vm);
  
  string config_file_name = vm["config"].as<string>();
  std::fstream config_file;
  config_file.open(config_file_name.c_str(), std::fstream::in);
  po::store(po::parse_config_file(config_file, config_file_options), vm);
  config_file.close();

  po::notify(vm);

  if (vm.count("help")) {
    std::cout << visible_options << std::endl;
    return 1;
  }

  bool save_patches = vm.count("patches");
  if (save_patches) {
    patch_dir = vm["patches"].as<string>();
  }

  RTreeClassifier classifier;
  std::vector< IplImage* > sources;
  std::vector< BaseKeypoint > base_set;
  Rng rng(seed);
  
  PatchGenerator make_patch(NULL, rng);
  make_patch.addWhiteNoise(vm.count("noise") > 0);
  make_patch.setThetaBounds(theta_min, theta_max);
  make_patch.setPhiBounds(phi_min, phi_max);
  make_patch.setLambdaBounds(lambda_min, lambda_max);
  
  if (vm.count("load-trees")) {
    // Load trees from file
    classifier.read( vm["load-trees"].as<string>().c_str() );
  }
  else if (vm.count("source")) {
    // Train new classifier on set of source images
    const std::vector<string> &source_files = vm["source"].as< std::vector<string> >();
    std::map< const IplImage*, string > source_map;

    // Load source images
    printf("Loading source images\n");
    sources.resize(source_files.size());
    for (int i = 0; i < (int)source_files.size(); ++i) {
      string name = source_files[i];
      IplImage *image = cvLoadImage(name.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
      sources[i] = image;
      fs::path image_path = fs::complete(name);
      source_map[image] = image_path.string();
    }

    // Detect keypoints in all images
    printf("Detecting keypoints\n");
    BOOST_FOREACH( IplImage* img, sources ) {
      std::vector<Keypoint> keys;
      if (vm.count("sift"))
        keys = siftKeypoints(img);
      else
        keys = starKeypoints(img, 7, 0);

      KeepBestPoints(keys, samples);
      
      BOOST_FOREACH( Keypoint &key, keys ) {
        base_set.push_back( BaseKeypoint(key.x, key.y, img) );
      }
    }
    
    // Randomly select the base set
    printf("Selecting base set\n");
    base_set = selectBaseSet(base_set, classes, rng);

    if (vm.count("save-base-set")) {
      std::ofstream base_set_file( vm["save-base-set"].as<string>().c_str() );
      BOOST_FOREACH( BaseKeypoint &key, base_set ) {
        string image_path = source_map[key.image];
        base_set_file << key.x << ' ' << key.y << ' ' << image_path << std::endl;
      }
      base_set_file.close();
    }

    if (!vm.count("reduced"))
      reduced_classes = classes;
    
    printf("Training classifier\n");
    {
      Timer timer("Training time");
      classifier.train(base_set, rng, make_patch, trees, depth, views, reduced_classes);
    }
  }
  else {
    // Bad input
    std::cout << "Error! Must specify a trees file or at least one source image!"
              << std::endl << visible_options << std::endl;
    return 1;
  }

  if (vm.count("save-trees")) {
    const string &file_name = vm["save-trees"].as<string>();
    classifier.write(file_name.c_str());
    std::cout << "Wrote classifier to " << file_name << std::endl;
  }
  
  classifier.setThreshold(threshold);

  printf("Smoothing source images\n");
  BOOST_FOREACH( IplImage* source, sources ) {
    cvSmooth(source, source);
  }

  std::ofstream sig_file;
  bool save_sigs = vm.count("save-sigs");
  if (save_sigs)
    sig_file.open( vm["save-sigs"].as<string>().c_str() );

  // TODO: this test procedure only makes sense if classes == reduced_classes
  printf("Calculating performance\n");
  int size = RandomizedTree::PATCH_SIZE;
  int correct = 0;
  float* post;
  posix_memalign(reinterpret_cast<void**>(&post), 16, reduced_classes * sizeof(float));
  for (int i = 0; i < classes; ++i) {
    BaseKeypoint key = base_set[i];
    cv::WImageView1_b image(key.image);
    cv::WImageView1_b patch(&image, key.x - size/2, key.y - size/2, size, size);
    classifier.getSignature(patch.Ipl(), post);

    float max_prob = 0.0;
    int best_class = -1;
    for (int c = 0; c < reduced_classes; ++c) {
      float prob = post[c];
      if (prob > max_prob) {
        max_prob = prob;
        best_class = c;
      }
    }

    if (save_sigs) {
      for (int c = 0; c < reduced_classes; ++c)
        sig_file << post[c] << ' ';
      sig_file << std::endl;
    }

    if (save_patches) {
      char file_name[128];
      sprintf(file_name, "%s/base%i.pgm", patch_dir.c_str(), i);
      cvSaveImage(file_name, patch.Ipl());
    }

    if (best_class == i) {
      ++correct;
      printf("Y");
    } else {
      printf("N");
    }
  }
  free(post);

  printf("\nCorrect: %i\n", correct);

  if (save_sigs)
    sig_file.close();
  
  BOOST_FOREACH( IplImage* img, sources )
    cvReleaseImage(&img);
  
  return 0;
}
