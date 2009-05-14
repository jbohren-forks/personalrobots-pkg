/*
 * Copyright (C) 2009 Willow Garage
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NEVEN_HH
#define NEVEN_HH

#include <stdint.h>

#include <vector>

struct btk_FaceFinder;
struct btk_DCR;
struct btk_SDK;

namespace neven
{

  struct Face
  {
    float confidence;
    float midpointx;
    float midpointy;
    float eyedist;
  };

  enum ProcessingConfig { CONFIG_SPEED, CONFIG_STANDARD, CONFIG_PRECISION };

  class FaceDetector
  {
    btk_FaceFinder* fd_;
    btk_SDK* sdk_;
    btk_DCR* dcr_;

  public:
    FaceDetector(uint32_t max_width, uint32_t max_height, uint32_t max_faces,
                 uint32_t min_face_width, uint32_t max_face_width, ProcessingConfig conf);
    ~FaceDetector();

    std::vector<Face> findFaces(char* img, uint32_t width, uint32_t height);

  };
}

#endif
