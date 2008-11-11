#!/bin/bash


cd `rospack find executive_trex_pr2`


mkdir -p $1
export TREX_LOG_DIR=$1
sleep 5

cd $2
`rospack find executive_trex_pr2`/bin/trexfast `rospack find executive_trex_pr2`/cfg/nav.cfg
