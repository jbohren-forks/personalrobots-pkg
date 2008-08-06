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

#include <gtest/gtest.h>
#include <ros/service.h>
#include <std_srvs/StaticMap.h>

const unsigned int g_valid_bmp_res = 0.1;
const unsigned int g_valid_bmp_width = 10;
const unsigned int g_valid_bmp_height = 10;

int g_argc;
char** g_argv;

class MapClientTest : public testing::Test
{
  private:
    // A node is needed to make a service call
    ros::node* n;

  protected:
    void SetUp()
    {
      ros::init(g_argc, g_argv);
      n = new ros::node("map_client_test");
    }
    void TearDown()
    {
      ros::fini();
      delete n;
    }
};

/* Try to retrieve the map via a valid PNG file.  Succeeds if no 
 * exception is thrown, if the call succeeds, and if
 * the loaded image matches the known dimensions of the map.  */
TEST_F(MapClientTest, retrieve_valid_bmp)
{
  try
  {
    std_srvs::StaticMap::request  req;
    std_srvs::StaticMap::response resp;
    bool call_result = ros::service::call("static_map", req, resp);
    ASSERT_TRUE(call_result);
    ASSERT_FLOAT_EQ(resp.map.resolution, g_valid_bmp_res);
    ASSERT_EQ(resp.map.width, g_valid_bmp_width);
    ASSERT_EQ(resp.map.height, g_valid_bmp_height);
  }
  catch(...)
  {
    FAIL() << "Uncaught exception : " << "This is OK on OS X";
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  return RUN_ALL_TESTS();
}
