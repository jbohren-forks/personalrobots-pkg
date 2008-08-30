#ifndef __COORDS_H__
#define __COORDS_H__

/**
   @brief Simple 2d coordinate class
 */
class coord2d {
 public:
  coord2d(int xa, int ya) { 
    x = xa;
    y = ya;
  };

  int x;
  int y;
};

/**
   @brief Wrapper for a list of 2d coordinates
 */
class coordList {
 public:
  void add(const coord2d& coord) {
    coords.push_back(coord);
  };

  std::vector<coord2d> coords;
};

#endif
