/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "resource_retriever/retriever.h"
#include <ros/package.h>
#include <ros/console.h>

namespace resource_retriever
{

Retriever::Retriever()
{

}

Retriever::~Retriever()
{

}

MemoryResource Retriever::get(const std::string& url)
{
  if (url.find("package://") == 0)
  {
    std::string file = url;
    file.erase(0, strlen("package://"));
    size_t pos = file.find("/");
    if (pos != std::string::npos)
    {
      std::string package = file.substr(0, pos);
      file.erase(0, pos);
      std::string package_path = ros::package::getPath(package);
      file = package_path + file;

      FILE* f = fopen(file.c_str(), "r");
      if (f)
      {
        fseek(f, 0, SEEK_END);
        uint32_t size = ftell(f);
        fseek(f, 0, SEEK_SET);

        MemoryResource r;
        r.size = size;
        r.data.reset(new uint8_t[size]);

        if (fread(r.data.get(), 1, size, f) != size)
        {
          fclose(f);
          ROS_ERROR("Error reading from file [%s]", file.c_str());
          return MemoryResource();
        }

        fclose(f);

        return r;
      }
    }
  }

  return MemoryResource();
}

}
