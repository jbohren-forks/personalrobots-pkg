#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include <string>
#include <vector>
#include <math.h>
using namespace std;

void split(const string &s, vector<string> &t, const string &d)
{
  size_t start = 0, end;
  while ((end = s.find_first_of(d, start)) != string::npos)
  {
    t.push_back(s.substr(start, end-start));
    start = end + 1;
  }
  t.push_back(s.substr(start));
}

int main(int argc, char **argv)
{
  string bg, fg;
  vector<string> fgs;
  FILE *log = fopen("log.txt", "w");
  if (argc < 3)
  {
    printf("give at least two filenames.\n");
    //bg = string("test_bg.jpg");
    //fg = string("test_fg.jpg");
    bg = string("../../../stair__sharks_testdata/stapler_pen_apple/shark_00001_-024.99886_.jpg");
    fgs.push_back(string("../../../stair__sharks_testdata/stapler_pen_apple/shark_00500_-000.05039_.jpg"));
    fgs.push_back(string("../../../stair__sharks_testdata/stapler_pen_apple/shark_00501_-000.00508_.jpg"));
  }
  else
  {
    bg = string(argv[1]);
    for (int i = 2; i < argc; i++)
      fgs.push_back(string(argv[i]));
  }

  SDL_Surface *screen, *bg_image, *fg_image;
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    printf("SDL init error: %s\n", SDL_GetError());
    return 1;
  }
  bg_image = IMG_Load(bg.c_str());
  if (!bg_image)
  {
    printf("SDL image load error: %s\n", IMG_GetError());
    SDL_Quit();
    return 1;
  }
  screen = SDL_SetVideoMode(bg_image->w, bg_image->h, 32, SDL_ANYFORMAT);
  if (!screen)
  {
    printf("SDL_SetVideoMode error: %s\n", SDL_GetError());
    SDL_FreeSurface(bg_image);
    SDL_FreeSurface(fg_image);
    SDL_Quit();
    return 1;
  }

  for (int f_idx = 0; f_idx < fgs.size(); f_idx++)
  {
    double lang = -1000;
    fg = fgs[f_idx];
    vector<string> tokens;
    split(fg, tokens, "_");
    if (tokens.size() < 2)
    {
      printf("woah! I expected at least two tokens separated by underscores in the filename\n");
      return 5;
    }
//    for (int i = 0; i < tokens.size(); i++)
//      printf("%d = [%s]\n", i, tokens[i].c_str());
    string lang_str = tokens[tokens.size() - 2];
//    printf("lang str = [%s]\n", lang_str.c_str());
    lang = atof(lang_str.c_str());
    printf("lang = %f\n", lang);

    fg_image = IMG_Load(fg.c_str());
    if (!fg_image)
    {
      printf("SDL image load error: %s\n", IMG_GetError());
      SDL_Quit();
      return 1;
    }
    //printf("loaded %s: %dx%d %dbpp\n", argv[1], image->w, image->h, image->format->BitsPerPixel);
    //printf("image bpp = %d\n", bg_image->format->BytesPerPixel);
    for (int y = 0; y < bg_image->h; y++)
    {
      double centroid = 0, sum = 0;
      for (int x = 0; x < bg_image->w; x++)
      {
        uint8_t *bg_p = (uint8_t *)bg_image->pixels + y*bg_image->pitch + x*3;
        uint8_t *fg_p = (uint8_t *)fg_image->pixels + y*fg_image->pitch + x*3;
        uint8_t bg_r = *(bg_p + bg_image->format->Rshift/8);
        uint8_t fg_r = *(fg_p + fg_image->format->Rshift/8);
        *(fg_p + fg_image->format->Gshift/8) = 0;
        *(fg_p + bg_image->format->Bshift/8) = 0;
        int diff = fg_r - bg_r;
        if (diff > 50)
        {
          *(fg_p + fg_image->format->Rshift/8) = diff;
          centroid += (diff) * x;
          sum += diff;
        }
        else
          *(fg_p + fg_image->format->Rshift/8) = 0;
      }
      centroid /= sum;
      if (sum > 200)
      {
        //printf("centroid = %f sum = %f\n", centroid, sum);
        int x = (int)floor(centroid);
        if (x < 0) x = 0;
        if (x >= fg_image->w) x = fg_image->w - 1;
        uint8_t *fg_p = (uint8_t *)fg_image->pixels + y*fg_image->pitch + x*3;
        *(fg_p + fg_image->format->Gshift/8) = 255;
        fprintf(log, "%f %d %f %f\n", lang, y, centroid, sum);
      }
    }

    SDL_BlitSurface(fg_image, 0, screen, 0);
    SDL_Flip(screen);
    SDL_FreeSurface(fg_image);
  }
  bool done = false;
  SDL_Event event;
  while (!done && SDL_WaitEvent(&event) != -1)
  {
    switch(event.type)
    {
      case SDL_KEYDOWN:
      case SDL_QUIT:
        done = true;
        break;
      case SDL_VIDEOEXPOSE:
        SDL_BlitSurface(fg_image, 0, screen, 0);
        SDL_Flip(screen);
        break;
      default:
        break;
    }
  }
  SDL_FreeSurface(bg_image);
  SDL_Quit();
  fclose(log);

  return 0;
}

