/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <stdexcept> // for std::runtime_error
#include <gtest/gtest.h>
#include "map_server/image_loader.h"

const char* g_invalid_image_file = "foo";

/* Note that these must be changed if the test image changes */
const char* g_valid_png_file = "test/testmap.png";
const unsigned int g_valid_png_width = 10;
const unsigned int g_valid_png_height = 10;

/* Note that these must be changed if the test image changes */
const char* g_valid_bmp_file = "test/testmap.bmp";
const unsigned int g_valid_bmp_width = 10;
const unsigned int g_valid_bmp_height = 10;

/* Try to load a valid PNG file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions of the file. 
 *
 * This test can fail on OS X, due to an apparent limitation of the
 * underlying SDL_Image library. */
TEST(map_server, load_valid_png)
{
  try
  {
    std_srvs::StaticMap::response map_resp;
    map_server::loadMapFromFile(&map_resp, g_valid_png_file, 0.1, false);
    ASSERT_FLOAT_EQ(map_resp.map.resolution, 0.1);
    ASSERT_EQ(map_resp.map.width, g_valid_png_width);
    ASSERT_EQ(map_resp.map.height, g_valid_png_height);
  }
  catch(...)
  {
    FAIL() << "Uncaught exception : " << "This is OK on OS X";
  }
}

/* Try to load a valid BMP file.  Succeeds if no exception is thrown, and if
 * the loaded image matches the known dimensions of the file. */
TEST(map_server, load_valid_bmp)
{
  try
  {
    std_srvs::StaticMap::response map_resp;
    map_server::loadMapFromFile(&map_resp, g_valid_bmp_file, 0.1, false);
    ASSERT_FLOAT_EQ(map_resp.map.resolution, 0.1);
    ASSERT_EQ(map_resp.map.width, g_valid_bmp_width);
    ASSERT_EQ(map_resp.map.height, g_valid_bmp_height);
  }
  catch(...)
  {
    FAIL() << "Uncaught exception";
  }
}

/* Try to load an invalid file.  Succeeds if a std::runtime_error exception
 * is thrown. */
TEST(map_server, load_invalid_file)
{
  try
  {
    std_srvs::StaticMap::response map_resp;
    map_server::loadMapFromFile(&map_resp, "foo", 0.1, false);
  }
  catch(std::runtime_error &e)
  {
    SUCCEED();
    return;
  }
  catch(...)
  {
    FAIL() << "Uncaught exception";
  }
  FAIL() << "Didn't throw exception as expected";
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
