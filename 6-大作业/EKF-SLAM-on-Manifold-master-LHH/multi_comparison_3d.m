% comparison of following methods in 3D environment
% - FEKF SLAM 
% - FEJ-EKF SLAM
% - EKF SLAM
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
niter = 50;
nsteps = 400;
se3EKF_RMS.position = [];
se3EKF_RMS.orientation = [];
se3EKF_NEES.pose = [];
se3EKF_NEES.orientation = [];

notREKF_RMS.position = [];
notREKF_RMS.orientation = [];
notREKF_NEES.pose = [];
notREKF_NEES.orientation = [];

REKF_RMS.position = [];
REKF_RMS.orientation = [];
REKF_NEES.pose = [];
REKF_NEES.orientation = [];

FEKF_RMS.position = [];
FEKF_RMS.orientation = [];
FEKF_NEES.pose = [];
FEKF_NEES.orientation = [];

EKF_RMS.position = [];
EKF_RMS.orientation = [];
EKF_NEES.pose = [];
EKF_NEES.orientation = [];

for i = 1:niter
    tic;
    data = gen_data(0);
    while max(data.state(:,4) )<499.9
       data = gen_data( 0 );
    end        
    save(['data/', int2str(i)], 'data');
    %load('./data.mat');
    nposes = size(data.poses.position, 2);
    nlandmarks = size(data.landmarks, 1);
    fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);
  
    FEKF_result = FEKF_SLAM( data );
    fprintf('F-EKF:\n, it is %d -th loop',i);
    [FEKF_RMS.position(i, :), FEKF_RMS.orientation(i, :), FEKF_NEES.pose(i, :), FEKF_NEES.orientation(i, :)] = ...
        FEKF_plot_rms_nees( FEKF_result, data, 0 );  
    %%
    REKF_result = REKF_SLAM( data );
    fprintf('RI-EKF:\n, it is %d -th loop',i);
    [REKF_RMS.position(i, :), REKF_RMS.orientation(i, :), REKF_NEES.pose(i, :), REKF_NEES.orientation(i, :)] = ...
        REKF_plot_rms_nees( REKF_result, data, 0 );
    
    %%
    RocEKF_result = RocEKF_SLAM( data );
    fprintf('Roc-EKF:\n, it is %d -th loop',i);
    [RocEKF_RMS.position(i, :), RocEKF_RMS.orientation(i, :), RocEKF_NEES.pose(i, :), RocEKF_NEES.orientation(i, :)] = ...
        RocEKF_plot_rms_nees( RocEKF_result, data, 0 );
    %%
    notREKF_result =  notREKF_SLAM( data );
    fprintf(' notR-EKF:\n, it is %d -th loop',i);
    [ notREKF_RMS.position(i, :),  notREKF_RMS.orientation(i, :),  notREKF_NEES.pose(i, :),  notREKF_NEES.orientation(i, :)] = ...
         notREKF_plot_rms_nees(  notREKF_result, data, 0 );
   %%
    se3EKF_result =  se3EKF_SLAM( data );
        fprintf(' SE(3)-EKF:\n, it is %d -th loop',i);
    [ se3EKF_RMS.position(i, :),  se3EKF_RMS.orientation(i, :),  se3EKF_NEES.pose(i, :),  se3EKF_NEES.orientation(i, :)] = ...
         se3EKF_plot_rms_nees(  se3EKF_result, data, 0 );
   %%
    EKF_result = EKF_SLAM( data );
    fprintf('SO(3)-EKF:\n, it is %d -th loop',i);
    [EKF_RMS.position(i, :), EKF_RMS.orientation(i, :), EKF_NEES.pose(i, :), EKF_NEES.orientation(i, :)] = ...
        EKF_plot_rms_nees( EKF_result, data, 0 );
    toc;
    
end

[EKF_RMS_ave, EKF_NEES_ave] = compute_rms_nees_ave( EKF_RMS, EKF_NEES );
[FEKF_RMS_ave, FEKF_NEES_ave] = compute_rms_nees_ave( FEKF_RMS, FEKF_NEES );
[notREKF_RMS_ave, notREKF_NEES_ave] = compute_rms_nees_ave( notREKF_RMS, notREKF_NEES );
[REKF_RMS_ave, REKF_NEES_ave] = compute_rms_nees_ave( REKF_RMS, REKF_NEES );
[RocEKF_RMS_ave, RocEKF_NEES_ave] = compute_rms_nees_ave( RocEKF_RMS, RocEKF_NEES );
[se3EKF_RMS_ave, se3EKF_NEES_ave] = compute_rms_nees_ave( se3EKF_RMS, se3EKF_NEES );

plot1()
