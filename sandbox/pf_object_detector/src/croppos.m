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
function [im, box] = croppos(im, box)

% [newim, newbox] = croppos(im, box)
% Crop positive example to speed up latent search.

% crop image around bounding box
pad = 0.5*((box(3)-box(1)+1)+(box(4)-box(2)+1));
x1 = max(1, round(box(1) - pad));
y1 = max(1, round(box(2) - pad));
x2 = min(size(im, 2), round(box(3) + pad));
y2 = min(size(im, 1), round(box(4) + pad));

im = im(y1:y2, x1:x2, :);
box([1 3]) = box([1 3]) - x1 + 1;
box([2 4]) = box([2 4]) - y1 + 1;
