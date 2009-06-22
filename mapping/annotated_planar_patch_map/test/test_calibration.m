D='/u/sorokin/ros/ros-pkg/mapping/annotated_planar_patch_map/test_data/';

f1=[D '1242970355.977350558.txt'];
f2=[D '1242970363.935062558.txt'];

pts1=load(f1);
pts2=load(f2);

v=find((pts1(:,1)~=0));
%v=v(1:10:end);
plot3(pts1(v,1),pts1(v,2),pts1(v,3),'x')

v2=find((pts2(:,1)~=0));
%v2=v2(1:10:end);
hold on
plot3(pts2(v2,1),pts2(v2,2),pts2(v2,3),'rx')
hold off

%%

v=find((pts1(:,1)~=0));
%v=v(1:10:end);
plot(pts1(v,1),pts1(v,2),'x')

v2=find((pts2(:,1)~=0));
%v2=v2(1:10:end);
hold on
plot(pts2(v2,1),pts2(v2,2),'rx')
hold off

%%
v=find((pts1(:,1)~=0));

g1_members=abs(pts1(v,3))<0.2;
sum(g1_members)

x0=mean(pts1(v(find(g1_members)),:),1);
oX0=pts1(v(find(g1_members)),:)-repmat(x0,sum(g1_members),1);
[pu,ps,pv]=svd(oX0'*oX0);
plot3(oX0(:,1),oX0(:,2),oX0(:,3),'x')

dir3=pv(:,3);
diag(ps)

g1_members=abs(oX0*dir3)<0.02;
oX0=oX0(g1_members,:);
x0=mean(oX0,1);
oX0=oX0-repmat(x0,sum(g1_members),1);
[pu,ps,pv]=svd(oX0'*oX0);
diag(ps)

hold on
plot3(oX0(:,1),oX0(:,2),oX0(:,3),'rx')
hold off




%%

point1=floor(rand()*numel(v))+1;

x1=pts1(v(point1),:);
d=pts1(v,:)-repmat(x1,numel(v),1);
close_match1=find(sum(d.*d,2)<0.1);
numel(close_match1)

d=pts2(v2,:)-repmat(x1,numel(v2),1);
close_match2=find(sum(d.*d,2)<0.1);
numel(close_match2)


part1Points=pts1(v(close_match1),:);

x0=mean(part1Points,1);
part1PointsCentered=part1Points-repmat(x0,size(part1Points,1),1);
[pu,ps,pv]=svd(part1PointsCentered'*part1PointsCentered);


figure(2)
plot3(part1PointsCentered(:,1),part1PointsCentered(:,2),part1PointsCentered(:,3))

d=pv(:,3)
f_set=find(abs(part1PointsCentered*d)<median(abs(part1PointsCentered*d)))

figure(3)
plot3(part1PointsCentered(f_set,1),part1PointsCentered(f_set,2),part1PointsCentered(f_set,3))


hold on 
plot3(pts2(v2(close_match),1),pts2(v2(close_match),2),pts2(v2(close_match),3),'yo')
hold off

plane1=