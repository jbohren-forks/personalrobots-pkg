#include "integral.h"
#include <cassert>

// source: uchar, WxH
// tilted: int, W+1xH+1
// flat_tilted: int, W+1xH+1
void TiltedIntegral(IplImage* source, IplImage* tilted, IplImage* flat_tilted)
{
    assert(source && source->depth == (int)IPL_DEPTH_8U);
    assert(tilted && tilted->depth == (int)IPL_DEPTH_32S);
    assert(flat_tilted && flat_tilted->depth == (int)IPL_DEPTH_32S);

    uchar* src = (uchar*) source->imageData;
    int src_step = source->widthStep / sizeof(uchar);
    int* tilt = (int*) tilted->imageData;
    int tilt_step = tilted->widthStep / sizeof(int);
    int* flat = (int*) flat_tilted->imageData;
    int flat_step = flat_tilted->widthStep / sizeof(int);
    int width = source->width;

    // Fill row 0 with zeros
    memset(tilt, 0, (width + 1) * sizeof(int) );
    memset(flat, 0, (width + 1) * sizeof(int) );
    tilt += tilt_step;
    flat += flat_step;

    // Fill in row 1 explicitly
    tilt[0] = 0;
    flat[0] = src[0];
    
    for (int x = 1; x < width; ++x) {
        tilt[x] = src[x-1];
        flat[x] = src[x] + src[x-1];
    }

    tilt[width] = flat[width] = src[width-1];
    src += src_step;
    tilt += tilt_step;
    flat += flat_step;

    // Fill in remaining rows by dynamic programming
    for (int y = 2; y < tilted->height; ++y, src += src_step,
             tilt += tilt_step, flat += flat_step) {
        tilt[0] = tilt[-tilt_step + 1];

        int elem = tilt[-tilt_step + 2] + src[-src_step] + src[0];
        tilt[1] = flat[0] = elem;
        flat[1] = flat[-tilt_step + 2] + src[-src_step] + src[1] + src[0];
        
        for (int x = 2; x < width; ++x) {
            tilt[x] = tilt[-tilt_step + x - 1] + tilt[-tilt_step + x + 1] -
                tilt[-tilt_step*2 + x] + src[-src_step + x - 1] + src[x - 1];
            flat[x] = flat[-flat_step + x - 1] + flat[-flat_step + x + 1] -
                flat[-flat_step*2 + x] + src[x] + src[x - 1];
        }

        elem = tilt[-tilt_step + width - 1] + src[-src_step + width - 1] + src[width - 1];
        tilt[width] = flat[width] = elem;
    }
}
