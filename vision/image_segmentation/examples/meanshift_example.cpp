#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>

#include <image_segmentation/meanshift_generic.h>

using namespace std;

// --------------------------------------------------------------
/*!
 * \brief Creates PointCloud datastructure from file
 *
 * File format: x y z label 2
 */
// --------------------------------------------------------------
unsigned int loadPointCloud(string filename, double** data)
{
  // ----------------------------------------------
  // Open file
  ifstream infile(filename.c_str());
  if (infile.is_open() == false)
  {
    return -1;
  }

  // ----------------------------------------------
  // Count number of lines in the file
  unsigned int nbr_samples = 0;
  char line[256];
  while (infile.getline(line, 256))
  {
    nbr_samples++;
  }

  infile.clear();
  infile.seekg(ios::beg);

  // ----------------------------------------------
  // Resize containers appropriately
  *data = (double*)malloc(nbr_samples * 3 * sizeof(double));

  // ----------------------------------------------
  // Read file
  // file format: x y z label 2
  unsigned int tempo;
  for (unsigned int i = 0 ; i < nbr_samples ; i++)
  {
    infile >> (*data)[i*3 + 0];
    infile >> (*data)[i*3 + 1];
    infile >> (*data)[i*3 + 2];
    infile >> tempo;
    infile >> tempo;
  }

  infile.close();
  return nbr_samples;
}


int main()
{
  /*
  int n = 100000;
  int d = 3;
  double poop[n * d];
  for (int i = 0 ; i < n ; i++)
  {
    for (int j = 0 ; j < d ; j++)
    {
      poop[i*d + j] = static_cast<double>(rand() % 10);
    }
  }
  */
  double* data = NULL;
  int n = loadPointCloud("pt_cloud_260.xyz_label_conf", &data);
  int d = 3;

  double labels[n];
  double* means = (double*) malloc(d * n * sizeof(double));

  double bandwidth = 0.37;
  int max_iter = 2;
  double rate = 0.4;

  time_t start,end;
  time(&start);
  clustering::meanshiftGeneric(data, d, n, bandwidth, rate, max_iter, labels, means);
  time(&end);
  cout << "clustering took this long: " << difftime (end,start) << endl;

/*
  for (int i = 0 ; i < n ; i++)
  {
    cout << data[i*3 + 0] << " " << data[i*3 + 1] << " " << data[i*3 + 2] << " "<< labels[i] << endl;
  }
*/
}
