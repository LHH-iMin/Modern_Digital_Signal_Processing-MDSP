% comparison of following methods in 3D environment
% - REKF SLAM 
% - FEJ-EKF SLAM
% - T-EKF-SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('f_ekf_3dTest_mod/');
addpath('se3_ekf_3d/');
addpath('robotcentric_ekf_3d_mod/');
addpath('right_ekf_3d_mod/');
addpath('not_right_ekf_3d/');
addpath('ekf_3d_mod/');
addpath('lie_utils/');
% generate simulation data
data = gen_data(0);
%load('data.mat');
nposes = size(data.poses.position, 2);
nlandmarks = size(data.landmarks, 1);
fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);
%%
fprintf('EKF\n');
EKF_result = EKF_SLAM( data );
% EKF_plot_estimation( EKF_result, data );
% EKF_plot_rms_nees( EKF_result, data, 1 );

fprintf('F-EKF\n');
FEKF_result = FEKF_SLAM( data );
% %FEKF_plot_estimation( FEKF_result, data );
% FEKF_plot_rms_nees( FEKF_result, data, 1 );


fprintf('notR-EKF-EKF\n');
notREKF_result = notREKF_SLAM( data );
% %REKF_plot_estimation( REKF_result, data );
% notREKF_plot_rms_nees( notREKF_result, data, 0 );

fprintf('R-EKF-mod\n');
REKF_result = REKF_SLAM( data );
% REKF_plot_estimation( REKF_result, data );
% REKF_plot_rms_nees( REKF_result, data, 1 );


fprintf('RobotCentric-EKF\n');
RocEKF_result = RocEKF_SLAM( data );
% %RocEKF_plot_estimation( RocEKF_result, data );
% RocEKF_plot_rms_nees( RocEKF_result, data, 1 );

fprintf('se3-EKF\n');
se3EKF_result = se3EKF_SLAM( data );
% %REKF_plot_estimation( REKF_result, data );
% se3EKF_plot_rms_nees( se3EKF_result, data, 1 );





%%情景A
for i=1: size(REKF_result,2)
    EKF_detP_theta(i)=det(EKF_result{i}.cov(1:3,1:3));
    FEKF_detP_theta(i)=det(FEKF_result{i}.cov(1:3,1:3));
    notREKF_detP_theta(i)=det(notREKF_result{i}.cov(1:3,1:3));
    REKF_detP_theta(i)=det(REKF_result{i}.cov(1:3,1:3));
    RocEKF_detP_theta(i)=det(RocEKF_result{i}.cov(1:3,1:3));
    se3EKF_detP_theta(i)=det(se3EKF_result{i}.cov(1:3,1:3));
end


EKF_orientation(1)= 0;
FEKF_orientation(1)=0;
notREKF_orientation(1)=0;
REKF_orientation(1)=0;
RocEKF_orientation(1)=0;
se3EKF_orientation(1)=0;
for i=2: size(REKF_result,2)
    EKF_orientation(i)= acos((trace(EKF_result{i-1}.orientation*(EKF_result{i}.orientation)') - 1) / 2);
    FEKF_orientation(i)=acos((trace(FEKF_result{i-1}.orientation*(FEKF_result{i}.orientation)') - 1) / 2);
    notREKF_orientation(i)=acos((trace(notREKF_result{i-1}.orientation*(notREKF_result{i}.orientation)') - 1) / 2);
    REKF_orientation(i)=acos((trace(REKF_result{i-1}.orientation*(REKF_result{i}.orientation)') - 1) / 2);
    RocEKF_orientation(i)=acos((trace(RocEKF_result{i-1}.orientation*(RocEKF_result{i}.orientation)') - 1) / 2);
    se3EKF_orientation(i)=acos((trace(se3EKF_result{i-1}.orientation*(se3EKF_result{i}.orientation)') - 1) / 2);
end

%%情景B

for i=1: size(REKF_result,2)
    EKF_detP_r(i)=det(EKF_result{i}.cov(4:6,4:6));
    FEKF_detP_r(i)=det(FEKF_result{i}.cov(4:6,4:6));
    notREKF_detP_r(i)=det(notREKF_result{i}.cov(4:6,4:6));
    REKF_detP_r(i)=det(REKF_result{i}.cov(4:6,4:6));
    RocEKF_detP_r(i)=det(RocEKF_result{i}.cov(4:6,4:6));
    se3EKF_detP_r(i)=det(se3EKF_result{i}.cov(4:6,4:6));
end

for i=1: size(REKF_result,2)
    EKF_detP_pose(i)=log(det(EKF_result{i}.cov(1:6,1:6)));
    FEKF_detP_pose(i)=log(det(FEKF_result{i}.cov(1:6,1:6)));
    notREKF_detP_pose(i)=log(det(notREKF_result{i}.cov(1:6,1:6)));
    REKF_detP_pose(i)=log(det(REKF_result{i}.cov(1:6,1:6)));
    RocEKF_detP_pose(i)=log(det(RocEKF_result{i}.cov(1:6,1:6)));
    se3EKF_detP_pose(i)=log(det(se3EKF_result{i}.cov(1:6,1:6)));
end

%%
for i=1:1
%     figure
% plot(1: size(REKF_result,2),EKF_detP_theta,'r',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),FEKF_detP_theta,'g',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),notREKF_detP_theta,'b',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),REKF_detP_theta,'c',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),RocEKF_detP_theta,'m',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),se3EKF_detP_theta,'k',"LineWidth",1.5);hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
% xlabel('时间步长');ylabel('det(P_{\theta})');
% 
% figure
% plot(1: size(REKF_result,2),EKF_orientation,'r',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),FEKF_orientation,'g',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),notREKF_orientation,'b',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),REKF_orientation,'c',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),RocEKF_orientation,'m',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),se3EKF_orientation,'k',"LineWidth",1.5);hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
% xlabel('时间步长');ylabel('旋转角度\theta');
% %%
% figure;
% plot(1: size(REKF_result,2),EKF_detP_r,'r',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),FEKF_detP_r,'g',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),notREKF_detP_r,'b',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),REKF_detP_r,'c',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),RocEKF_detP_r,'m',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),se3EKF_detP_r,'k',"LineWidth",1.5);hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
% xlabel('时间步长');ylabel('log(det(P_{r}))');
% 
% figure;
% plot(1: size(REKF_result,2),EKF_detP_pose,'r',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),FEKF_detP_pose,'g',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),notREKF_detP_pose,'b',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),REKF_detP_pose,'c',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),RocEKF_detP_pose,'m',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),se3EKF_detP_pose,'k',"LineWidth",1.5);hold on;
% legend('SO(3)-EKF','FEJ-EKF','notR-EKF','RI-EKF', 'Roc-EKF','SE(3)-EKF' ,'Location','northeast');
% xlabel('时间步长');ylabel('log(det(P_{pose}))');
% end
% 

end
%%
% 第一个图的六个子图
for i=1:1
    figure;
% 第一个子图
subplot(2,3,1);
plot(1: size(REKF_result,2),EKF_detP_theta,'r',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('SO(3)-EKF');

% 第二个子图
subplot(2,3,2);
plot(1: size(REKF_result,2),FEKF_detP_theta,'g',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('FEJ-EKF');

% 第三个子图
subplot(2,3,3);
plot(1: size(REKF_result,2),notREKF_detP_theta,'b',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('notR-EKF');

% 第四个子图
subplot(2,3,4);
plot(1: size(REKF_result,2),REKF_detP_theta,'c',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('RI-EKF');

% 第五个子图
subplot(2,3,5);
plot(1: size(REKF_result,2),RocEKF_detP_theta,'m',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('Roc-EKF');

% 第六个子图
subplot(2,3,6);
plot(1: size(REKF_result,2),se3EKF_detP_theta,'k',"LineWidth",1.5);
ylim([0 0.15]);yticks([0 0.05 0.1 0.15]);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('det(P_{\theta})');
title('SE(3)-EKF');
end
%% 第二个图的六个子图
for i=2:2
    figure;
% 第一个子图
subplot(2,3,1);
plot(1: size(REKF_result,2),EKF_orientation,'r',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('SO(3)-EKF');

% 第二个子图
subplot(2,3,2);
plot(1: size(REKF_result,2),FEKF_orientation,'g',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('FEJ-EKF');

% 第三个子图
subplot(2,3,3);
plot(1: size(REKF_result,2),notREKF_orientation,'b',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('notR-EKF');

% 第四个子图
subplot(2,3,4);
plot(1: size(REKF_result,2),REKF_orientation,'c',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('RI-EKF');

% 第五个子图
subplot(2,3,5);
plot(1: size(REKF_result,2),RocEKF_orientation,'m',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('Roc-EKF');

% 第六个子图
subplot(2,3,6);
plot(1: size(REKF_result,2),se3EKF_orientation,'k',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');
ylabel('旋转角度\theta');
title('SE(3)-EKF');
end

%% 第三个图的六个子图
for i=3:3

figure;
% 第一个子图
subplot(2,3,1);
plot(1: size(REKF_result,2),EKF_detP_r,'r',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('SO(3)-EKF');

% 第二个子图
subplot(2,3,2);
plot(1: size(REKF_result,2),FEKF_detP_r,'g',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('FEJ-EKF');

% 第三个子图
subplot(2,3,3);
plot(1: size(REKF_result,2),notREKF_detP_r,'b',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('notR-EKF');

% 第四个子图
subplot(2,3,4);
plot(1: size(REKF_result,2),REKF_detP_r,'c',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('RI-EKF');

% 第五个子图
subplot(2,3,5);
plot(1: size(REKF_result,2),RocEKF_detP_r,'m',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('Roc-EKF');

% 第六个子图
subplot(2,3,6);
plot(1: size(REKF_result,2),se3EKF_detP_r,'k',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('det(P_{r})');
title('SE(3)-EKF');
end

%%
% 第四个图的六个子图
for i=4:4
    figure;
% 第一个子图
subplot(2,3,1);
plot(1: size(REKF_result,2),EKF_detP_pose,'r',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('SO(3)-EKF');

% 第二个子图
subplot(2,3,2);
plot(1: size(REKF_result,2),FEKF_detP_pose,'g',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('FEJ-EKF');

% 第三个子图
subplot(2,3,3);
plot(1: size(REKF_result,2),notREKF_detP_pose,'b',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('notR-EKF');

% 第四个子图
subplot(2,3,4);
plot(1: size(REKF_result,2),REKF_detP_pose,'c',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('RI-EKF');

% 第五个子图
subplot(2,3,5);
plot(1: size(REKF_result,2),RocEKF_detP_pose,'m',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('Roc-EKF');

% 第六个子图
subplot(2,3,6);
plot(1: size(REKF_result,2),se3EKF_detP_pose,'k',"LineWidth",1.5);
xlim([1 size(REKF_result,2)]);
xlabel('时间步长');ylabel('log(det(P_{pose}))');
title('SE(3)-EKF');
end

% % 第三个子图
% figure
% plot(1: size(REKF_result,2),notREKF_detP_pose,'b',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),REKF_detP_pose,'c',"LineWidth",1.5);hold on;
% plot(1: size(REKF_result,2),RocEKF_detP_pose,'m',"LineWidth",1.5);hold on;
% xlim([1 size(REKF_result,2)]);xlabel('时间步长');ylabel('log(det(P_{pose}))');
% legend('notR-EKF','R-EKF',"Roc-EKF")