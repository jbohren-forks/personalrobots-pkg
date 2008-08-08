if ~exist('d')
  d = load('../extract_laser/log.txt');
  fprintf('ok, loaded %d scanlines\n', size(d,1));
end
baseline = 0.49;
hres = 704;
vres = 480;
hfov = 50;
vfov = hfov * vres / hres;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = [];
r = 2000:length(d);
lang = -d(r,1) * pi/180 - 225*pi/180;
img_y = d(r,2);
img_x = d(r,3);
azi = ((hres/2-img_x) / hres * hfov + 90) * pi/180;
ele =  (vres/2-img_y) / vres * vfov * pi/180;
ray = [cos(ele).*cos(azi) sin(ele) cos(ele).*sin(azi)];
ln   = [-cos(lang), zeros(length(lang),1), -sin(lang)];
lorg = [baseline; 0; 0];
numer = baseline * ln(:,1);
denom = dot(ray',ln')';
t = numer ./ denom;
t = [t,t,t];
proj = t .* ray;

%plot_idx = 1:20:(length(proj));
%plot3(proj(plot_idx,1),proj(plot_idx,3),proj(plot_idx,2),'.');
%axis equal;

%save -ascii 'test.txt' proj

% in non-insane terms:
%proj = t .* ray
%for i=1:100:length(img_y)
%  t = lorg'*ln(i,:)' / (ray(i,:) * ln(i,:)');
%  proj = t * ray(i,:);
%  scene = [scene; proj];
%end

