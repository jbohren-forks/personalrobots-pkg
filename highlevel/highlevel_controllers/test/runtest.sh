#!/bin/bash

TMPFILE=`mktemp` || {
    echo "ERROR creating temporary file"
    exit 1
}

# set default parameters

STAGE_WORLD_FILE="willow-pr2-5cm.world"
MAP_WORLD_FILE="willow-full-0.05.pgm"
MAP_RESOLUTION="0.05"
PLANNER_TYPE="ADPlanner"
PLANNER_TIME="20.0"
ENVIRONMENT_TYPE="2D"
ENV3D_OBST_COST="lethal"

resolution="50"

# parse options

while [ ! -z "$1" ]; do
    case $1 in
	-h|--help)
echo "runtest.sh command line options:"
echo "  [-r|--resolution]  <mm>    map resolution in mm (50 or 25)"
echo "  [-p|--planner]     <type>  planner type (ad or ara)"
echo "  [-t|--time]        <sec>   time allowed for planning"
echo "  [-e|--environment] <type>  map type (2d or 3d)"
echo "  [-c|--cost]        <cost>  3D map obstacle threshold (lethal, inscribed, or circumscribed)"
        exit 0;;
	
	-r|--resolution)
	    resolution=$2
	    shift; shift; continue;;
	
	-p|--planner)
	    PLANNER_TYPE=$2
	    shift; shift; continue;;
	
	-t|--time)
	    if [ -z "$2" ]; then
		echo "ERROR -t option requires a parameter"
		exit 1
	    fi
	    PLANNER_TIME=$2
	    shift; shift; continue;;
	
	-e|--environment)
	    ENVIRONMENT_TYPE=$2
	    shift; shift; continue;;
	
	-c|--cost)
	    if [ -z "$2" ]; then
		echo "ERROR -c option requires a parameter"
		exit 1
	    fi
	    ENV3D_OBST_COST=$2
	    shift; shift; continue;;
	
	*)
	    echo "ERROR unhandled option(s) $*"
	    exit 1;;
    esac
done

# check some of the parameters

case $resolution in
    50)
	echo blah 50
	STAGE_WORLD_FILE="willow-pr2-5cm.world"
	MAP_WORLD_FILE="willow-full-0.05.pgm"
	MAP_RESOLUTION="0.05"
	continue;;
    25)
	echo blah 25
	STAGE_WORLD_FILE="willow-pr2-2.5cm.world"
	MAP_WORLD_FILE="willow-full-0.025.pgm"
	MAP_RESOLUTION="0.025"
	continue;;
    *)
	echo "ERROR invalid resolution: $resolution"
	exit 1;;
esac

case $PLANNER_TYPE in
    ad|AD|adstar|ADStar|adplanner|ADPlanner)
	PLANNER_TYPE="ADPlanner"
	continue;;
    ar|AR|ara|ARA|arastar|ARAStar|araplanner|ARAPlanner)
	PLANNER_TYPE="ARAPlanner"
	continue;;
    *)
	echo "ERROR invalid planner: $PLANNER_TYPE"
	exit 1;;
esac

case $ENVIRONMENT_TYPE in
    2d|2D)
	ENVIRONMENT_TYPE="2D"
	continue;;
    3d|3D|3dkin|3DKIN)
	ENVIRONMENT_TYPE="3DKIN"
	continue;;
    *)
	echo "ERROR invalid environment: $ENVIRONMENT_TYPE"
	exit 1;;
esac

cat testcase.xml.in \
    | sed -e s/@STAGE_WORLD_FILE@/$STAGE_WORLD_FILE/ \
          -e s/@MAP_WORLD_FILE@/$MAP_WORLD_FILE/ \
          -e s/@MAP_RESOLUTION@/$MAP_RESOLUTION/ \
          -e s/@PLANNER_TYPE@/$PLANNER_TYPE/ \
          -e s/@PLANNER_TIME@/$PLANNER_TIME/ \
          -e s/@ENVIRONMENT_TYPE@/$ENVIRONMENT_TYPE/ \
          -e s/@ENV3D_OBST_COST@/$ENV3D_OBST_COST/ \
    > $TMPFILE

roslaunch $TMPFILE

rm -f $TMPFILE
