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
function bbox = getboxes(model, boxes)

% bbox = getboxes(model, boxes)
% Predict bounding boxes from root and part locations.

if isempty(boxes)
  bbox = [];
else
  bbox = zeros(size(boxes,1), 5);
  for i = 1:size(boxes,1)
    A = [boxes(i,3)-boxes(i,1)];
    for j=1:4:size(boxes, 2)-2;
      A = [A boxes(i, j:j+1)];
    end
    c = boxes(i, end-1);
    bbox(i,:) = [A*model.components{c}.x1 ... 
                 A*model.components{c}.y1 ...
                 A*model.components{c}.x2 ...
                 A*model.components{c}.y2 ...
                 boxes(i, end)];
  end
end
