#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <string>
#include <fstream>
#include <exception>
#include <stdexcept>
#include <iostream>

// Interface to LXI device
#include "LxiInterface.h"

// from getopt
extern char *optarg;
extern int optind, opterr, optopt;

// global
bool quit=false;

using namespace std;

char const *progname=__FILE__;

void usage() {
  fprintf(
          stderr,
          "usage: %s <IPv4 address> [-f <filename>] \n"
          " command line terminal to LXI device \n"
          " <address> IP addresss of LXI device \n"
          " -f : record session to log file <filename>\n", 
          progname);
}
	
void catch_signal(int sig)
{
  if (!quit) {
    quit = true;
  } else {
    fprintf(stderr,"error, forcing exit\n");
    exit(1);
  }
}


int main(int argc, char** argv) {	
  progname = argv[0];
  if (argc < 2) {
    usage();
    return 1;
  }
  
  std::fstream log;
  
  int optchar;
  //int tmp;
  while ((optchar = getopt(argc, argv, "pM:NRh")) != -1) {		
    switch(optchar) { 
    case 'f': 
      if (log.is_open()) {
        cerr << "Log file is already open" << endl;
        return 1;
      } 
      log.open(optarg);
      if (!log.is_open()) {
        cerr << "Error opening log file '" << optarg << "'" << endl;
        return 1;
      }
      break;
    case 'h':
      usage();
      return 0;
    case '?':
      fprintf(stderr, "unknown/unsupported option %c\n", optopt);
      return 1;			   
    default:
      fprintf(stderr, "programming error %c = %d??\n", optchar, optchar);
      return 1;				
    } // end switch	   
  }

  if (argc < 2) {
    cerr << "Error, please provide IP address for LXI device" << endl;
    usage();
    return 1;
  }


  const char *ip_address = argv[1];
  Interface *interface = new LxiInterface(ip_address);
  if (interface == NULL) throw std::bad_alloc();
  interface->Open();              
      
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  signal(SIGHUP, catch_signal);

  string reply;
  while (!quit)
  {

    char *cmd = readline("> ");
    if (strlen(cmd) == 0) {
      interface->Read(reply);
    }
    else {      
      if (log.is_open()) log << cmd << endl;
      interface->WriteRead(cmd, reply);
    }    
    if (log.is_open()) log << reply << endl;
    cout << reply << endl;    

  }
    
  delete interface;
  interface = NULL;
    
  return 0;
}
