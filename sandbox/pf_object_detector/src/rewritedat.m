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

function rewritedat(datfile, inffile, hdrfile, I)

% rewritedat(datfile, inffile, hdrfile, I)
% Rewrite training files with a subset of the examples.
% Used to shrink the cache.

fhdr = fopen(hdrfile, 'rb');
header = fread(fhdr, 3, 'int32');
labelsize = header(2);
fclose(fhdr);

oldfile = [datfile '_tmp'];
unix(['mv ' datfile ' ' oldfile]);
fin = fopen(oldfile, 'rb');
fout = fopen(datfile, 'wb');

% sort indexes so we never have to seek before the current position
I = sort(I);

pos = 1;
for i = 1:length(I)
  cnt = I(i)-pos;
  while cnt > 0
    % + 2 to include the num non-zero blocks and example length
    info = fread(fin, labelsize+2, 'int32');
    dim = info(end);
    fseek(fin, dim*4, 0);
    cnt = cnt - 1;
  end
  y = fread(fin, labelsize+2, 'int32');
  dim = y(end);
  x = fread(fin, dim, 'single');
  fwrite(fout, y, 'int32');
  fwrite(fout, x, 'single');
  pos = I(i)+1;
end

fclose(fin);
fclose(fout);

% keep the info file in sync with the data file
oldfile = [inffile '_tmp'];
unix(['cp ' inffile ' ' oldfile]);
[labels, scores, unique] = readinfo(inffile);
labels = labels(I);
scores = scores(I);
unique = unique(I);
fid = fopen(inffile, 'w');
for i = 1:length(I)
  fprintf(fid, '%d\t%f\t%d\n', labels(i), scores(i), unique(i));
end
fclose(fid);
