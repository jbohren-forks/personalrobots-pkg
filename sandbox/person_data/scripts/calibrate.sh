#todo-- move the first 4 lines into a separate script shared with launch.sh

python `rospack find person_data`/common/checkrobot.py ${ROBOT} || exit

ssh ${ROBOT}2 mkdir -p /bags/person_data

ssh ${ROBOT}3 mkdir -p /bags/person_data

ssh ${ROBOT}4 mkdir -p /bags/person_data

roslaunch person_calibrate.launch
