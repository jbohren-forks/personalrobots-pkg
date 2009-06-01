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

function [boxes1, boxes2] = pascal_test(cls, model, testset, suffix)

% [boxes1, boxes2] = pascal_test(cls, model, testset, suffix)
% Compute bounding boxes in a test set.
% boxes1 are bounding boxes from root placements
% boxes2 are bounding boxes using predictor function

globals;
pascal_init;
ids = textread(sprintf(VOCopts.imgsetpath, testset), '%s');

% run detector in each image
try
  load([cachedir cls '_boxes_' testset '_' suffix]);
catch
  for i = 1:length(ids);
    fprintf('%s: testing: %s %s, %d/%d\n', cls, testset, VOCyear, ...
            i, length(ids));
    im = imread(sprintf(VOCopts.imgpath, ids{i}));  
    boxes = detect(im, model, model.thresh);
    if ~isempty(boxes)
      b1 = boxes(:,[1 2 3 4 end]);
      b1 = clipboxes(im, b1);
      boxes1{i} = nms(b1, 0.5);
      if length(model.partfilters) > 0
        b2 = getboxes(model, boxes);
        b2 = clipboxes(im, b2);
        boxes2{i} = nms(b2, 0.5);
      else
        boxes2{i} = boxes1{i};
      end
    else
      boxes1{i} = [];
      boxes2{i} = [];
    end
    showboxes(im, boxes1{i});
  end    
  save([cachedir cls '_boxes_' testset '_' suffix], 'boxes1', 'boxes2');
end
