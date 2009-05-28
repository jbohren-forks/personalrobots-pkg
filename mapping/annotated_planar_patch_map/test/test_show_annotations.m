root_dir='/u/sorokin/bags/run_may_21/dump/'

f1=[root_dir 'annotations2D.txt'];
f2=[root_dir 'polygons3D__map2D.txt'];
f3=[root_dir 'polygons3D__map3D.txt'];

a2d=load(f1);
map2d=load(f2);
map3d=load(f3);


figure(1)
clf
plot_polygons(a2d)
axis ij

valid_2dm=map2d(:,1)>=0 & map2d(:,1)<=640 & map2d(:,2)>=0 & map2d(:,2)<=480;
figure(3)
clf
plot_polygons(map2d(valid_2dm,:))
plot_polygons(a2d,'r')

axis ij



figure(2)
plot_polygons(map2d(:,:))



splits=[0;find(map2d(:,1)==0 & map2d(:,2)==0);];
clf
hold on
for iS=2:numel(splits)
  f=splits(iS-1)+1;
  t=splits(iS)-1;
  
 plot(map2d(f:t,1),map2d(f:t,2))
end
hold off

