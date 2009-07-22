python checkrobot.py ${ROBOT} || exit

ssh ${ROBOT}2 mkdir -p /bags/person_data

ssh ${ROBOT}3 mkdir -p /bags/person_data

ssh ${ROBOT}4 mkdir -p /bags/person_data

namedlaunch person_data.launch
