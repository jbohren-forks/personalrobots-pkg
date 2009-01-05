#ifndef PLACE_RECOGNITION_IO_H
#define PLACE_RECOGNITION_IO_H

#include <vector>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <Eigen/Core>

void dir(std::string full_path, std::vector<std::string>& res) 
{
  boost::filesystem::path path( full_path );
  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator dir_itr( path );
        dir_itr != end_itr;
        ++dir_itr )
  {
    res.push_back( dir_itr->path().leaf() );
  }
  std::sort(res.begin(), res.end() );
}

struct SiftGeo
{
  float x;
  float y;
  float scale;
  float angle;
  float affine[2][2];
  float cornerness;
  int dimension;
  unsigned char descriptor[128];
};

unsigned int countSiftGeo(const char* file)
{
  std::ifstream is(file, std::ios::binary);

  // get length of file
  is.seekg(0, std::ios::end);
  int length = is.tellg();
  is.seekg(0, std::ios::beg);

  return length / sizeof(SiftGeo);
}

std::vector<SiftGeo> readSiftGeo(const char* file)
{
  unsigned int size = countSiftGeo(file);

  // read data as a block
  std::ifstream is(file, std::ios::binary);
  std::vector<SiftGeo> entries(size);
  is.read((char*)(&entries[0]), size * sizeof(SiftGeo));
  is.close();

  return entries;
}

template< typename MatrixT >
void readSiftGeoDescriptors(const char* file, Eigen::MatrixBase<MatrixT>& descriptors)
{
  std::vector<SiftGeo> keypts = readSiftGeo(file);
  for (int i = 0; i < descriptors.rows(); ++i) {
    for (int j = 0; j < descriptors.cols(); ++j) {
      descriptors(i,j) = (float)(keypts[i].descriptor[j]);
    }
  }
}

#endif
