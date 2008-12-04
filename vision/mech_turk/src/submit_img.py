#!/usr/bin/python

import uuid,sys,os,urllib


target_img_name=uuid.uuid4();

print target_img_name

srv_name="vision-app1.cs.uiuc.edu"
#srv_name="128.174.241.84"
srv_port=":8080"
dest_user="submitimageuser"
target_session="people-poly-1"

inp_img=sys.argv[1];

cmd="scp -i id_rsa_SIU %s %s@%s:/var/datasets/%s/%s.jpg" % ( inp_img, dest_user, srv_name, target_session, target_img_name);
print cmd
os.system(cmd);

submit_url="http://%s%s/mt/newHIT/" % (srv_name,srv_port)

parameters={'session':target_session,
		'frame':str(target_img_name),
		'original_name':str(inp_img) };

print submit_url
response = urllib.urlopen(submit_url, urllib.urlencode(parameters)).read()
print response



