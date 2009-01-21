/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "setup.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

extern "C" {
#include <err.h>
}

using namespace ompl;
using namespace boost;
using namespace std;

static void cleanup();
static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void create_setup();

static std::string baseFilename();

static string costmapType;
static SBPLBenchmarkOptions opt;
static string travmapFilename;

static shared_ptr<SBPLBenchmarkSetup> setup;


int main(int argc, char ** argv)
{
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  parse_options(argc, argv);
  create_setup();		// also dumps costmap to file
}


void cleanup()
{
  setup.reset();
}


void usage(ostream & os)
{
  os << "options:\n"
     << "   -h               help (this message)\n"
     << "   -m  <name>       name of the costmap implementation\n"
     << "   -s  <name>       name of the setup\n"
     << "   -r  <cellsize>   set grid resolution\n"
     << "   -i  <in-radius>  set INSCRIBED radius\n"
     << "   -c  <out-radius> set CIRCUMSCRIBED radius\n"
     << "   -I  <inflate-r>  set INFLATION radius\n"
     << "   -d  <doorwidth>  set width of doors (office setups)\n"
     << "   -H  <hallwidth>  set width of hallways (office setups)\n"
     << "   -n  <filename>   Net PGM file to load (for -s pgm)\n"
     << "   -g  <gray>       cutoff for obstacles in PGM images (for -s pgm)\n"
     << "   -o  <filename>   specify alternative filename for output\n";
}


static string summarizeOptions()
{
  ostringstream os;
  os << "-s" << opt.name
     << "-m" << costmapType
     << "-r" << (int) rint(1e3 * opt.resolution)
     << "-i" << (int) rint(1e3 * opt.inscribed_radius)
     << "-c" << (int) rint(1e3 * opt.circumscribed_radius)
     << "-I" << (int) rint(1e3 * opt.inflation_radius)
     << "-d" << (int) rint(1e3 * opt.door_width)
     << "-H" << (int) rint(1e3 * opt.hall_width);
  return os.str();
}


std::string baseFilename()
{
  return "costmap-" + summarizeOptions();
}


static void sanitizeOptions()
{
  if (opt.inscribed_radius > opt.circumscribed_radius)
    opt.circumscribed_radius = opt.inscribed_radius;
  if (opt.inscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.inscribed_radius;
  if (opt.circumscribed_radius > opt.inflation_radius)
    opt.inflation_radius = opt.circumscribed_radius;
}


void parse_options(int argc, char ** argv)
{
  // default values for options
  costmapType = "costmap_2d";
  travmapFilename = "";
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      cerr << argv[0] << ": problem with option '" << argv[ii] << "'\n";
      usage(cerr);
      exit(EXIT_FAILURE);
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(cout);
	exit(EXIT_SUCCESS);
	
      case 's':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -s requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	opt.name = argv[ii];
 	break;
	
      case 'm':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -m requires a name argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	costmapType = argv[ii];
 	break;
	
      case 'r':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -r requires a cellsize argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.resolution;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading cellsize argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'i':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -i requires inscribed radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.inscribed_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading inscribed radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'c':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -c requires circumscribed radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.circumscribed_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading circumscribed radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'I':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -I requires inflation radius argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.inflation_radius;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading inflation radius argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'd':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -d requires a doorwidth argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.door_width;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading doorwidth argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'H':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -H requires a hallwidth argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.hall_width;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading hallwidth argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'n':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -n requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	opt.pgm_filename = argv[ii];
 	break;
	
      case 'g':
	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -g requires a gray argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	{
	  istringstream is(argv[ii]);
	  is >> opt.obstacle_gray;
	  if ( ! is) {
	    cerr << argv[0] << ": error reading gray argument from \"" << argv[ii] << "\"\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	}
 	break;
	
      case 'o':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -o requires a filename argument\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	travmapFilename = argv[ii];
 	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
  
  sanitizeOptions();
}


void create_setup()
{
  if ("costmap_2d" == costmapType)
    opt.use_sfl_cost = false;
  else if ("sfl" == costmapType)
    opt.use_sfl_cost = true;
  else
    errx(EXIT_FAILURE,
	 "create_setup(): unknown costmapType \"%s\", use costmap_2d or sfl",
	 costmapType.c_str());
  
  cout << "creating setup \"" << opt.name << "\"\n" << flush;
  if (travmapFilename.empty())
    travmapFilename = baseFilename() + ".txt";
  ofstream dump_os(travmapFilename.c_str());
  if ( ! dump_os)
    errx(EXIT_FAILURE,
	 "create_setup(): could not open travmap file %s",
	 travmapFilename.c_str());
  setup.reset(createBenchmark(opt, &cout, &dump_os));
  if ( ! setup)
    errx(EXIT_FAILURE, "could not create setup with name \"%s\"", opt.name.c_str());
}
