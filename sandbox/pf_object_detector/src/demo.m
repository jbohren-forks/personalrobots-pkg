%
%Copyright (C) 2008, 2009 Pedro Felzenszwalb, Ross Girshick
%Copyright (C) 2007 Pedro Felzenszwalb, Deva Ramanan
%
%Permission is hereby granted, free of charge, to any person obtaining
%a copy of this software and associated documentation files (the
%"Software"), to deal in the Software without restriction, including
%without limitation the rights to use, copy, modify, merge, publish,
%distribute, sublicense, and/or sell copies of the Software, and to
%permit persons to whom the Software is furnished to do so, subject to
%the following conditions:
%
%The above copyright notice and this permission notice shall be
%included in all copies or substantial portions of the Software.
%
%THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
%EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
%MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
%NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
%LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
%OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
%WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%

function demo()

test('000034.jpg', 'car');
test('000061.jpg', 'person');
test('000084.jpg', 'bicycle');

function test(name, cls)
% load and display image
im=imread(name);
load(['VOC2007/' cls '_final']);
boxes = detect(im, model, 0);

clf;
image(im);
axis equal; 
axis on;
disp('input image');
disp('press any key to continue'); pause;

% load and display model

visualizemodel(model);
disp([cls ' model']);
disp('press any key to continue'); pause;

% detect objects

top = nms(boxes, 0.5);
showboxes(im, top);
%print(gcf, '-djpeg90', '-r0', [cls '.jpg']);
disp('detections');
disp('press any key to continue'); pause;

% get bounding boxes
bbox = getboxes(model, boxes);
top = nms(bbox, 0.5);
bbox = clipboxes(im, top);
showboxes(im, bbox);
disp('bounding boxes');
disp('press any key to continue'); pause;
