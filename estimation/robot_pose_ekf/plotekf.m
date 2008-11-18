load odom_file.txt;
load corr_file.txt;
load extra_file.txt;


figure; hold on;
axis equal;
plot(odom_file(:,1), odom_file(:,2),'b');
plot(corr_file(:,1), corr_file(:,2),'r');
plot(extra_file(:,1), extra_file(:,2), 'g');
hold off;

error_odom = sqrt( (odom_file(1,1)-odom_file(end,1))^2 + (odom_file(1,2)-odom_file(end,2))^2 )
error_corr = sqrt( (corr_file(1,1)-corr_file(end,1))^2 + (corr_file(1,2)-corr_file(end,2))^2 )
