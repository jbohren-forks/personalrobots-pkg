#!/bin/bash


cd `rospack find exec_trex`


cp `rospack find exec_trex`/pr2.sim.cfg `rospack find exec_trex`/pr2.cfg

mkdir $1
export TREX_LOG_DIR=$1
sleep 5
`rospack find exec_trex`/exec_trex_o_rt $2
