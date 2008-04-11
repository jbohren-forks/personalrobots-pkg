#include "image_utils/ppm_wrapper.h"
#include <cstdio>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: create_chessboard WIDTH HEIGHT [SIZE]\n");
    printf("  where WIDTH, HEIGHT are the number of square, and\n");
    printf("  SIZE is the pixel length of the side of each square.\n");
    return 1;
  }

  int w = atoi(argv[1]), h = atoi(argv[2]);
  int s = 200;
  if (argc >= 4)
    s = atoi(argv[3]);
  printf("creating %d by %d board with %d-pixel squares\n", w, h, s);

  int img_width = w*s, img_height = h*s;
  uint8_t *img = new uint8_t[img_width*img_height*3];
  for (int y = 0; y < img_height; y++)
    for (int x = 0; x < img_width; x++)
    {
      uint8_t *p = img + y * img_width * 3 + x * 3;
      uint8_t xc = (uint8_t)(x / s) % 2;
      uint8_t yc = (uint8_t)(y / s) % 2;
      uint8_t  c = (xc ^ yc ? 255 : 0);
      p[0] = p[1] = p[2] = c;
    }

  PpmWrapper::write_file("board.ppm", img_width, img_height, "rgb24", img);
  delete[] img;

  return 0;
}
