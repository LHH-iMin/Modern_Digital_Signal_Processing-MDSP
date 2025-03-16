figure;
plot(1:size(EKF_RMS_ave.position, 2), EKF_RMS_ave.position, 'r',"LineWidth",1.5); hold on;
plot(1:size(FEKF_RMS_ave.position, 2), FEKF_RMS_ave.position, 'g',"LineWidth",1.5); hold on;
plot(1:size(notREKF_RMS_ave.position, 2), notREKF_RMS_ave.position, 'b',"LineWidth",1.5); hold on;
plot(1:size(REKF_RMS_ave.position, 2), REKF_RMS_ave.position, 'c',"LineWidth",1.5); hold on;
plot(1:size(RocEKF_RMS_ave.position, 2), RocEKF_RMS_ave.position, 'm',"LineWidth",1.5); hold on;
% plot(1:size(se3EKF_RMS_ave.position, 2), se3EKF_RMS_ave.position, 'k',"LineWidth",1.5); hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF' ,'Location','northeast');
title('RMS:位置');xlim([0,size(REKF_RMS_ave.position, 2)]);
ylabel('RMS:位置/m');
xlabel('步长');
figure;
%subplot(2,2,1);
plot(1:size(EKF_RMS_ave.orientation, 2), EKF_RMS_ave.orientation, 'r',"LineWidth",1.5); hold on;
plot(1:size(FEKF_RMS_ave.orientation, 2), FEKF_RMS_ave.orientation, 'g',"LineWidth",1.5); hold on;
plot(1:size(notREKF_RMS_ave.orientation, 2), notREKF_RMS_ave.orientation, 'b',"LineWidth",1.5); hold on;
plot(1:size(REKF_RMS_ave.orientation, 2), REKF_RMS_ave.orientation, 'c',"LineWidth",1.5); hold on;
plot(1:size(RocEKF_RMS_ave.orientation, 2), RocEKF_RMS_ave.orientation, 'm',"LineWidth",1.5); hold on;
% plot(1:size(se3EKF_RMS_ave.orientation, 2), se3EKF_RMS_ave.orientation, 'k',"LineWidth",1.5); hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF' ,'Location','northeast');
title('RMS:方向');xlim([0,size(REKF_RMS_ave.orientation, 2)]);
ylabel('RMS:方向/rad');
xlabel('步长');
figure;
%subplot(2,2,1);
plot(1:size(EKF_NEES_ave.orientation, 2), EKF_NEES_ave.orientation, 'r',"LineWidth",1.5); hold on;
plot(1:size(FEKF_NEES_ave.orientation, 2), FEKF_NEES_ave.orientation, 'g',"LineWidth",1.5); hold on;
plot(1:size(notREKF_NEES_ave.orientation, 2), notREKF_NEES_ave.orientation, 'b',"LineWidth",1.5); hold on;
plot(1:size(REKF_NEES_ave.orientation, 2), REKF_NEES_ave.orientation, 'c',"LineWidth",1.5); hold on;
plot(1:size(RocEKF_NEES_ave.orientation, 2), RocEKF_NEES_ave.orientation, 'm',"LineWidth",1.5); hold on;
% plot(1:size(se3EKF_NEES_ave.orientation, 2), se3EKF_NEES_ave.orientation, 'k',"LineWidth",1.5); hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF' ,'Location','northeast');
title('NEES:方向');xlim([0,size(REKF_NEES_ave.orientation, 2)]);
ylabel('NEES:方向');
xlabel('步长');

figure;
plot(1:size(EKF_NEES_ave.pose, 2), EKF_NEES_ave.pose, 'r',"LineWidth",1.5); hold on;
plot(1:size(FEKF_NEES_ave.pose, 2), FEKF_NEES_ave.pose, 'g',"LineWidth",1.5); hold on;
% plot(1:size(notREKF_NEES_ave.pose, 2), notREKF_NEES_ave.pose, 'b',"LineWidth",1.5); hold on;
plot(1:size(REKF_NEES_ave.pose, 2), REKF_NEES_ave.pose, 'c',"LineWidth",1.5); hold on;
% plot(1:size(RocEKF_NEES_ave.pose, 2), RocEKF_NEES_ave.pose, 'm',"LineWidth",1.5); hold on;
% plot(1:size(se3EKF_NEES_ave.pose, 2), se3EKF_NEES_ave.pose, 'k'); hold on;
% plot(1:size(RocEKF_NEES_ave.pose, 2), 1.12*ones( 1,size(RocEKF_NEES_ave.pose, 2)  ), 'r--',"LineWidth",1.5); hold on;
% plot(1:size(RocEKF_NEES_ave.pose, 2), 0.89*ones( 1,size(RocEKF_NEES_ave.pose, 2)  ), 'r--',"LineWidth",1.5); hold on;
legend('SO(3)-EKF','FEJ-EKF','RI-EKF' ,'Location','northeast');% ,'95% confidence bound'
% legend('SO(3)-EKF','FEJ-EKF','RI-EKF', 'Roc-EKF' ,'Location','northeast');
%hleg=legend('R-EKF', '95% confidence bound' , 'Location','northwest');
xlim([0,size(REKF_NEES_ave.pose, 2)]);
ylabel('NEES:位姿');
xlabel('步长');
title('NEES:位姿');



fprintf('Mean values:\n');
fprintf('           RMS-Position    RMS-Orientation     NEES-Pose   NEES-Orientation\n');
fprintf('SO(3)-EKF            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(EKF_RMS_ave.position), mean(EKF_RMS_ave.orientation), mean(EKF_NEES_ave.pose), mean(EKF_NEES_ave.orientation));
fprintf('FEJ-EKF          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(FEKF_RMS_ave.position), mean(FEKF_RMS_ave.orientation), mean(FEKF_NEES_ave.pose), mean(FEKF_NEES_ave.orientation));
fprintf('notR-EKF          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(notREKF_RMS_ave.position), mean(notREKF_RMS_ave.orientation), mean(notREKF_NEES_ave.pose), mean(notREKF_NEES_ave.orientation));
fprintf('RI-EKF         %.5f            %.5f       %.5f           %.5f\n', ...
    mean(REKF_RMS_ave.position), mean(REKF_RMS_ave.orientation), mean(REKF_NEES_ave.pose), mean(REKF_NEES_ave.orientation));
fprintf('Roc-EKF            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(RocEKF_RMS_ave.position), mean(RocEKF_RMS_ave.orientation), mean(RocEKF_NEES_ave.pose), mean(RocEKF_NEES_ave.orientation));
fprintf('SE(3)-EKF            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(se3EKF_RMS_ave.position), mean(se3EKF_RMS_ave.orientation), mean(se3EKF_NEES_ave.pose), mean(se3EKF_NEES_ave.orientation));
