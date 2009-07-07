#Author: Ian Goodfellow
#File: Makefile
#Description: Makefile for stairvision ROS package. Checks out latest version of STAIR Vision Library and makes it. Based on the makefile for the opencv_latest package.

all: installed

SVN_DIR = build/stairvision-svn
SVN_URL = https://stairvision.svn.sourceforge.net/svnroot/stairvision/release/2.1
EIGEN_LINK =${SVN_DIR}/external/Eigen
LOCAL_MAKEFILE =${SVN_DIR}/make.local

include ${shell rospack find mk}/svn_checkout.mk

installed : ${SVN_DIR} ${EIGEN_LINK} ${LOCAL_MAKEFILE}
	cd ${SVN_DIR} && make external && make svllibs && make svlapps && cd ../..

${EIGEN_LINK} :
	-ln -s `rospack find eigen`/build/eigen2/Eigen ${EIGEN_LINK}

${LOCAL_MAKEFILE} :
	cp make.local ${LOCAL_MAKEFILE}

clean :
	cd ${SVN_DIR} && make clean && cd ../..
