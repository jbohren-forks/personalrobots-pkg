#octave version that plots graphs of gripper grasps
function grasp_grasping(filelist,info_csv)
figure(1)
hold on
figure(2)
hold on
figure(3)
hold on
id = fopen(filelist);
id_info = fopen(info_csv);
currInfo = fgets(id_info);
currInfo = fgets(id_info); #if there's a header
currFile = fgets(id);
i = 0;
while currFile ~= -1
  i = i + 1;
  plot_bool = 1;
  plot_str = '-@';
  currInfo2 = split(currInfo,",");
  currFile = deblank(currFile);
  data1 = load(currFile,'-force');
  currFile = fgets(id)
  currFile = deblank(currFile);
  data2 = load(currFile,'-force');
  currFile = fgets(id)
  currFile = deblank(currFile);
  data3 = load(currFile,'-force');
  if strcmp(strrep(deblank(currInfo2(1,:)), "\"",""),"odwalla")
    disp("o")
    plot_str = strcat(plot_str,'o');
    plot_bool = 1;
    
  elseif strcmp(strrep(deblank(currInfo2(1,:)), "\"",""),"naked")
    disp("x")
    plot_str = strcat(plot_str,'x');
    
  elseif strcmp(strrep(deblank(currInfo2(1,:)), "\"",""),"can")
    disp("+")
    plot_str = strcat(plot_str,'+');
    
  elseif strcmp(strrep(deblank(currInfo2(1,:)), "\"",""),"water")
    disp("*")
    plot_str = strcat(plot_str,'*');
    
  else
    disp("none")
  endif
  
  if index(strrep(deblank(currInfo2(2,:)), "\"",""),"full") & index(strrep(deblank(currInfo2(3,:)), "\"",""),"closed")
    disp("fc")
    plot_str = strcat(plot_str,'1'); #red
  
  elseif index(strrep(deblank(currInfo2(2,:)), "\"",""),"full") & index(strrep(deblank(currInfo2(3,:)), "\"",""),"open")
    disp("fo")
    plot_str = strcat(plot_str,'3'); #blue
    
  elseif index(strrep(deblank(currInfo2(2,:)), "\"",""),"empty") & index(strrep(deblank(currInfo2(3,:)), "\"",""),"closed")
    disp("ec")
    plot_str = strcat(plot_str,'6'); #brown (white)
    
  elseif index(strrep(deblank(currInfo2(2,:)), "\"",""),"empty") & index(strrep(deblank(currInfo2(3,:)), "\"",""),"open")
    disp("eo")
    plot_str = strcat(plot_str,'4'); #magenta
    
  elseif index(strrep(deblank(currInfo2(2,:)), "\"",""),"full") & index(strrep(deblank(currInfo2(3,:)), "\"",""),"sealed")
    disp("fs")
    plot_str = strcat(plot_str,'5'); #cyan
  endif
  
  if plot_bool
    figure(1)
    plot3(data1(:,5), data1(:,1), data1(:,3),plot_str);
    figure(2)
    plot3(data2(:,5), data2(:,1), data2(:,3),plot_str);
    figure(3)
    plot3(data3(:,5), data3(:,1), data3(:,3),plot_str);
  endif
  currFile = fgets(id)
  currInfo = fgets(id_info);
endwhile
figure(1)
xlabel('time')
ylabel('distance')
zlabel('force')
figure(2)
xlabel('time')
ylabel('distance')
zlabel('force')
figure(3)
xlabel('time')
ylabel('distance')
zlabel('force')
#h = ['-@o'; '-@x'; '-@+'; '-@*']
#M = ['odwalla'; 'naked'; 'can'; 'water']
#legend(h,M)
