#!/bin/bash
python `rospack find person_data`/common/checkrobot.py ${ROBOT} || exit
export bagconfig=`rospack find person_data`/config/machine2bagdir.txt
ssh ${ROBOT}2 mkdir -p `cat ${bagconfig}`

#ssh ${ROBOT}3 mkdir -p /bags/person_data

#ssh ${ROBOT}4 mkdir -p /bags/person_data

roslaunch `rospack find person_data`/data_collector_components/person_data.launch
