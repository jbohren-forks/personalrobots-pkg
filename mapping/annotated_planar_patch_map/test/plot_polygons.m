function plot_polygons(map2d,c)

if ~exist('c','var')
  c='b';
end

splits=[0;find(map2d(:,1)==0 & map2d(:,2)==0);];
hold on
for iS=2:numel(splits)
  f=splits(iS-1)+1;
  t=splits(iS)-1;
  if t>f
    plot(map2d(f:t,1),map2d(f:t,2),c)
  end
end
hold off
