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
function model = parsemodel(model, blocks)

% parsemodel(model, blocks)
% Update model parameters from weight vector representation.

% update root filters
for i = 1:length(model.rootfilters)
  s = size(model.rootfilters{i}.w);
  width1 = ceil(s(2)/2);
  width2 = floor(s(2)/2);
  s(2) = width1;
  f = reshape(blocks{model.rootfilters{i}.blocklabel}, s);
  model.rootfilters{i}.w(:,1:width1,:) = f;
  model.rootfilters{i}.w(:,width1+1:end,:) = flipfeat(f(:,1:width2,:));
end

% update offsets
for i = 1:length(model.offsets)
  model.offsets{i}.w = blocks{model.offsets{i}.blocklabel};
end

% update part filters and deformation models
for i = 1:length(model.partfilters)
  if model.partfilters{i}.fake
    continue;
  end

  model.defs{i}.w = reshape(blocks{model.defs{i}.blocklabel}, ...
                            size(model.defs{i}.w));
  partner = model.partfilters{i}.partner;
  if partner == 0
    % part is self-symmetric
    s = size(model.partfilters{i}.w);
    width1 = ceil(s(2)/2);
    width2 = floor(s(2)/2);
    s(2) = width1;
    f = reshape(blocks{model.partfilters{i}.blocklabel}, s);
    model.partfilters{i}.w(:,1:width1,:) = f;
    model.partfilters{i}.w(:,width1+1:end,:) = flipfeat(f(:,1:width2,:));
  else
    % part has a symmetric partner
    f = reshape(blocks{model.partfilters{i}.blocklabel}, ...
                size(model.partfilters{i}.w));
    model.partfilters{i}.w = f;
    model.partfilters{partner}.w = flipfeat(f);
    % flip linear term in horizontal deformation model
    model.defs{partner}.w = model.defs{i}.w;
    model.defs{partner}.w(2) = -1*model.defs{partner}.w(2);
  end
end
