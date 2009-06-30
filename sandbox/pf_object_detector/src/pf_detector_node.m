# /*
#  * Software License Agreement (BSD License)
#  *
#  *  Copyright (c) 2008, Willow Garage, Inc.
#  *  All rights reserved.
#  *
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions
#  *  are met:
#  *
#  *   * Redistributions of source code must retain the above copyright
#  *     notice, this list of conditions and the following disclaimer.
#  *   * Redistributions in binary form must reproduce the above
#  *     copyright notice, this list of conditions and the following
#  *     disclaimer in the documentation and/or other materials provided
#  *     with the distribution.
#  *   * Neither the name of Willow Garage, Inc. nor the names of its
#  *     contributors may be used to endorse or promote products derived
#  *     from this software without specific prior written permission.
#  *
#  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  *  POSSIBILITY OF SUCH DAMAGE.
#  %*/
#
# Author: Alexander Sorokin 
function pf_detector_node(tag,model_file,interval,scale,do_display)

%These are examples to run it:
% Fast
%pf_detector_node('person','/wg/stor5/sorokin/ros/ros-pkg/sandbox/pf_object_detector/models/VOC2008/person_final.mat','3','0.5')
% Accurate
%pf_detector_node('person','/wg/stor5/sorokin/ros/ros-pkg/sandbox/pf_object_detector/models/VOC2008/person_final.mat','10','1.0')

rosoct();
rosoct_add_msgs('sensor_msgs');
rosoct_add_msgs('rosoct');
rosoct_add_msgs('people');

global gModel;
global gScale
global gInterval
global gTag;
global bDisplay



node_name='person_detector_slow_pfHOG';
if ~exist('tag','var')
  gTag = rosoct_get_param([node_name '/tag'])
else
  gTag=tag;
end


if ~exist('model_file','var')
  model_file = rosoct_get_param([node_name '/model_file'])
end

load(model_file)

if ~exist('do_display','var')
   bDisplay = rosoct_get_param([node_name '/do_display'])
 else
   bDisplay=str2num(do_display);
end


gModel=model;
if ~exist('scale','var')
   gScale = rosoct_get_param([node_name '/scale'])
 else
   gScale=str2num(scale)
end
if ~exist('interval','var')
   gInterval = rosoct_get_param([node_name '/interval'])
else
   gInterval=str2num(interval)
end

gModel.interval=gInterval;



suc = rosoct_advertise('objects_2d',@people_PositionMeasurement, 1);
suc = rosoct_advertise('objects_2d_str',@rosoct_String, 1);

rosoct_subscribe(['/stereo/left/image'], @sensor_msgs_Image, @run_detector_on_msg, 1);

while 1
  numprocessed = rosoct_worker()
  pause(0.3)
end


function run_detector_on_msg(msg)


global gModel
global gScale
global gTag
global bDisplay

'got message'
%msg.uint8_data.layout
width = msg.uint8_data.layout.dim{1}.size;
height = msg.uint8_data.layout.dim{2}.size;
I = reshape(msg.uint8_data.data,[height width])';
I=cat(3,I,I,I);

if bDisplay
figure(1)
imshow(I)
size(I)
figure(2)
end


bboxes=detect_on_img(I, gModel, gScale);
%disp(bboxes)
strOut=''
for iB=1:size(bboxes,1)
  s=[gTag ' ' sprintf('%g ',bboxes(iB,:))]
  if iB>1
     strOut=[strOut ',' s];
  else
     strOut=s;
  end
  w=bboxes(iB,3);
  h=bboxes(iB,4);

  m=people_PositionMeasurement();
  m.header=msg.header;
  m.name=gTag;
  m.object_id=gTag;
  m.pos.x=bboxes(iB,1)+w/2;
  m.pos.y=bboxes(iB,2)+h/2;
  m.pos.z=1;
  m.covariance(1:9)=0;
  m.covariance(9)=1;
  m.covariance(1)=w/4;
  m.covariance(5)=h/4;
  m.reliability=bboxes(iB,5);
  m.initialization=0;

  rosoct_publish('objects_2d',m);
end
if size(bboxes,1)>0
  m = rosoct_String();
  m.data = strOut;
		      
  rosoct_publish('objects_2d_str',m);
end








