% linedata = SegmentLines(points, maxrange, show, pointthresh)
%
% uses split and merge algorithm
% points - a 2xN array of polar points (radius and angle)
% linedata - 4xK lines where first to rows are

% Software License Agreement (BSD License)
% Copyright (c) 2008, Rosen Diankov
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%   * Redistributions of source code must retain the above copyright notice,
%     this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the distribution.
%   * Neither the name of Stanford University nor the names of its
%     contributors may be used to endorse or promote products derived from
%     this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
function linedata = SegmentLines(points, maxrange, show, pointthresh)

if( ~exist('show','var') )
    show = 0;
end

if( ~exist('pointthresh','var') )
    pointthresh = 1;
end

thresh = 0.025;
linedata = [];

% dists = polar_points(1,:);
% angles = polar_points(2,:);
%points = [cos(angles).*dists; sin(angles).*dists];

if( exist('maxrange','var') )
    % remove all poinst greater than maxrange
    dists = sum(points.^2);
    Ibad = find(dists > maxrange*maxrange);
    start = 1;
    S = {};
    for bad = Ibad
        if( start < bad )
            S{end+1} = start:(bad-1);
        end
        start = bad+1;
    end

    if( start <= size(points,2) )
        S{end+1} = start:size(points,2);
    end
else
    S{1} = [1:size(points,2)];
end

i = 1;
while(i <= length(S))
    
    I = S{i};
    
    if( length(I) == 1 )
        % destroy
        S{i} = [];
        continue;
    end
    if( length(I) == 2 )
        i = i + 1;
        continue;
    end
    
    localpts = points(:,I);
    N = size(localpts,2);
    L = FitLine(localpts);
    
    % find the error from lines
    dists = abs(L(1:2)'*localpts + L(3));
    
    meandist = mean(abs(dists));
    distthresh = max(meandist, thresh);

    % split and start
    split = 0;
    rejected = 0;

    while(1)
        [d, ind] = max(dists);

        if( d > thresh )
            % check if outlier
            nextval = dists(min(length(dists),ind+1));
            prevval = dists(max(1, ind-1));

            if( ind < length(dists) & nextval > distthresh )
                
                % get the greatest split
                if( ind > 1 & prevval>thresh & prevval > nextval )
                    split = ind-1;
                else
                    split = ind;
                end

                break;
            elseif( ind > 1 & prevval>distthresh )
                split = ind-1;
                break;
            else
                % remove
                rejected = 1;
                dists(ind) = [];
                I(ind) = [];
            end
        else
            break;
        end
    end

    if( split > 0 )
        if( split == 1 )
            % reject first point
            S{i} = I(2:end);
        elseif( split == N )
            % reject last
            S{i} = I(1:(end-1));
        else
            Smove = S((i+1):end);
            S{i} = I(1:split);
            S{i+1} = I(split:end);
            S((i+2):(i+1+length(Smove))) = Smove;
        end
    elseif( rejected )
        if( length(I) >= 2 )
            S{i} = I; % have to redo
        else
            S{i} = []; % erase
        end
    else
        % good, add to array and advance
        i = i + 1;
    end
end

if( length(S) == 0 )
    return;
end

% merge collinear segments
allindices = S{1};
i = 2;
while(i <= length(S))

    newindices = [allindices S{i}];
    L = FitLine(points(:,newindices));
    dists = abs(L(1:2)'*points(:,newindices) + L(3));

    if( any(dists > thresh) )
        % make all indices into a line
        pts = points(:,allindices([1 end]));
        %pts = pts-L(1:2)*(L(1:2)'*pts-L(3));

        if( length(allindices) >= pointthresh & norm(pts(:,1)-pts(:,2)) > thresh )
            linedata = [linedata pts];
        end
        allindices = S{i};
    else
        % add to line
        allindices = newindices;
    end
    i = i + 1;
end

if( length(allindices) > 0 )
    pts = points(:,allindices([1 end]));
    if( length(allindices) >= pointthresh & norm(pts(:,1)-pts(:,2)) > thresh )
        linedata = [linedata pts];
    end
end

linedata = reshape(linedata, [4 size(linedata,2)/2]);

if( show )
    figure(1); clf();    
    plot(points(1,:), points(2,:), 'r.'); hold on;
    for i = 1:size(linedata,2)
        plot(linedata([1 3],i), linedata([2 4], i), 'b', 'linewidth',3);
    end
    axis equal;
    drawnow;
end

function [L] = FitLine(points, W)

if( exist('W','var') )
    Wtotal = sum(W);
    meanpts = (points*transpose(W))/Wtotal;
    cpoints = repmat(meanpts, [1 size(points,2)]) - points;
    N = -2 * sum(W.*cpoints(1,:).*cpoints(2,:));
    D = sum(W.*(cpoints(2,:).^2-cpoints(1,:).^2));
else
    meanpts = mean(points,2);
    cpoints = repmat(meanpts, [1 size(points,2)]) - points;
    N = -2 * sum(cpoints(1,:).*cpoints(2,:));
    D = sum((cpoints(2,:).^2-cpoints(1,:).^2));
end

angle = 0.5 * atan2(N, D);

c = cos(angle);
s = sin(angle);
r = -(meanpts(1)*c + meanpts(2)*s);

L = [c; s; r];

% Check out "Feature Extraction and Scene Interpretation for Map-Based
% Navigation and Map Building" by Arras adn Siegwart
% find variance (a pain)
% cangles = cos(polar_points(2,:));
% sangles = sin(polar_points(2,:));
% cangles2 = cangles.^2 - sangles.^2;
% sangles2 = 2 .* cangles .* sangles;
% 
% dAngle_dP = W.*(N * (meanpts(1)*cangles-meanpts(2)*sangles-polar_points(1,:).*cangles2)-...
%     D*(meanpts(1)*sangles+meanpts(2)*cangles-polar_points(1,:).*sangles2))/ (N^2+D^2);
% dR_dP = W./Wtotal*cos(polar_points(2,:)-angle) + dAngle_dP.*(meanpts(2)*c-meanpts(1)*s);
% 
% variance = zeros(2,2);
% variance(1,1) = sum(dAngle_dP.^2);
% variance(2,2) = sum(dR_dP.^2);
% variance(2,1) = sum(dAngle_dP.*dR_dP);
% variance(1,2) = variance(2,1);
