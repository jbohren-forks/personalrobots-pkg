#ifndef FAST_H
#define FAST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int x, y; } xy; 
typedef struct { int x, y, r; } xyr; 
typedef unsigned char byte;

xyr*  fast_nonmax(const byte* im, int xsize, int ysize, xyr* corners, int numcorners, int barrier, int* numnx);
xyr*  fast_corner_detect_9(const byte* im, int xsize, int ysize, int barrier, int* numcorners);
xy*  fast_corner_detect_10(const byte* im, int xsize, int ysize, int barrier, int* numcorners);
xy*  fast_corner_detect_11(const byte* im, int xsize, int ysize, int barrier, int* numcorners);
xy*  fast_corner_detect_12(const byte* im, int xsize, int ysize, int barrier, int* numcorners);

#ifdef __cplusplus
}
#endif

#endif
