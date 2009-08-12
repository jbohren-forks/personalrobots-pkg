/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSkeletonManager.h>
#include <OGRE/OgreMeshSerializer.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreResourceGroupManager.h>
#include <OGRE/OgreMath.h>
#include <OGRE/OgreDefaultHardwareBufferManager.h>
#include <OGRE/OgreManualObject.h>

#include "ogre_tools/stl_loader.h"

#include <boost/filesystem.hpp>

#include <ros/console.h>

/**
 * @file
 *
 * The stl_to_mesh binary converts a binary STL file to an Ogre mesh.  See http://en.wikipedia.org/wiki/STL_(file_format)#Binary_STL for a description of
 * the file format.
 *
 * @par Usage
 @verbatim
 $ stl_to_mesh <stl files> <output directory>
 $ stl_to_mesh <stl file> <output file>
 @endverbatim
 * See http://pr.willowgarage.com/wiki/STL_To_Ogre_Mesh_Converter for more information
 */

using namespace Ogre;
using namespace ogre_tools;
namespace fs=boost::filesystem;


int main( int argc, char** argv )
{
  if ( argc < 3 )
  {
    ROS_INFO( "Usage: stl_to_mesh <stl files> <output directory>" );
    ROS_INFO( "or     stl_to_mesh <stl file> <output file>" );

    return 0;
  }

  typedef std::vector<std::string> V_string;
  V_string inputFiles;
  V_string outputFiles;
  std::string outputDirectory = argv[ argc - 1 ];
  if ( outputDirectory.rfind( ".mesh" ) != std::string::npos )
  {
    ROS_INFO( "Converting single mesh: %s to %s", argv[1], outputDirectory.c_str() );
    inputFiles.push_back( argv[ 1 ] );
    outputFiles.push_back( outputDirectory );
  }
  else
  {
    ROS_INFO( "Converting multiple meshes, into output directory %s...", outputDirectory.c_str() );

    for ( int i = 1; i < argc - 1; ++i )
    {
      std::string inputFile = argv[ i ];
      inputFiles.push_back( inputFile );

      fs::path p(inputFile);
      if (!(p.extension() == ".stl" || p.extension() == ".STL" || p.extension() == ".stlb" || p.extension() == ".STLB"))
      {
        ROS_ERROR( "Input file %s is not a .stl or .STL file!", inputFile.c_str() );
        exit(1);
      }

      p = p.replace_extension("mesh");

      std::string outputFile = outputDirectory + "/" + p.filename();

      outputFiles.push_back( outputFile );
    }
  }

  // NB some of these are not directly used, but are required to
  //   instantiate the singletons used in the dlls
  LogManager* logMgr = 0;
  Math* mth = 0;
  MaterialManager* matMgr = 0;
  SkeletonManager* skelMgr = 0;
  MeshSerializer* meshSerializer = 0;
  DefaultHardwareBufferManager *bufferManager = 0;
  MeshManager* meshMgr = 0;
  ResourceGroupManager* rgm = 0;

  try
  {
    logMgr = new LogManager();
    logMgr->createLog( "STLToMesh_Ogre.log", false, false, true );
    rgm = new ResourceGroupManager();
    mth = new Math();
    meshMgr = new MeshManager();
    matMgr = new MaterialManager();
    matMgr->initialise();
    skelMgr = new SkeletonManager();
    meshSerializer = new MeshSerializer();
    bufferManager = new DefaultHardwareBufferManager(); // needed because we don't have a rendersystem

    for ( size_t i = 0; i < inputFiles.size(); ++i )
    {
      std::string inputFile = inputFiles[ i ];
      std::string outputFile = outputFiles[ i ];

      STLLoader loader;
      if (!loader.load(inputFile))
      {
        exit( 1 );
      }

      ROS_INFO( "Converting %s to %s...", inputFile.c_str(), outputFile.c_str() );
      ROS_INFO( "%d triangles", loader.triangles_.size() );
      std::stringstream ss;
      ss << "converted" << i;
      Ogre::MeshPtr mesh = loader.toMesh(ss.str());
      meshSerializer->exportMesh( mesh.get(), outputFile, Serializer::ENDIAN_LITTLE );
    }
  }
  catch ( Exception& e )
  {
    ROS_ERROR( "%s", e.what() );
  }

  delete meshSerializer;
  delete skelMgr;
  delete matMgr;
  delete meshMgr;
  delete bufferManager;
  delete mth;
  delete rgm;
  delete logMgr;

  return 0;
}
