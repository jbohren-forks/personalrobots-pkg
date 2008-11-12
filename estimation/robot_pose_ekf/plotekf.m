

load corr_file.txt;
load odom_file.txt;

figure; hold on;
plot(odom_file(:,1), odom_file(:,2),'b');
plot(corr_file(:,1), corr_file(:,2),'r');
hold off;
