% PLOTPR    Plot PR Curve Generated from scoreVideoResults
%           Assumes data input files have a single header line
%           followed by comma delimited threshold, recall and
%           precision.
%
% STAIR VISION PROJECT
% Copyright (c) 2008, Stanford University
%
% Stephen Gould <sgould@stanford.edu>

function [pr] = plotPR(filename, varargin);

pr = dlmread(filename, ',', 1, 0);
pr = sortrows(pr, 2);

plot(pr(:, 2), pr(:, 3), 'b', 'LineWidth', 2);
axis([0, 1, 0, 1]);
axis square;
xlabel('recall'); ylabel('precision'); grid on;
[maxf1, argf1] = max(2.0 * pr(:, 2) .* pr(:, 3) ./ (pr(:, 2) + pr(:, 3)));
disp(['Max. F1-score: ', num2str(maxf1), ' at t=', num2str(pr(argf1, 1))]);

if (nargin > 1),
    hold on;
    for i = 1:length(varargin),
        vpr = dlmread(varargin{i}, ',', 1, 0);
        plot(vpr(:, 2), vpr(:, 3), 'r', 'LineWidth', 2);
        [maxf1, argf1] = max(2.0 * vpr(:, 2) .* vpr(:, 3) ./ (vpr(:, 2) + vpr(:, 3)));
        disp(['Max. F1-score: ', num2str(maxf1), ' at t=', num2str(pr(argf1, 1))]);
    end;
    hold off;
    legend({filename, varargin{:}});
end;
