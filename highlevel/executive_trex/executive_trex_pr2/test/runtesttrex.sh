#!/bin/bash


cd `rospack find executive_trex_pr2`


mkdir $1
export TREX_LOG_DIR=$1
sleep 5

cd $2
`rospack find executive_trex_pr2`/build/trex_fast `rospack find executive_trex_pr2`/cfg/nav.cfg
