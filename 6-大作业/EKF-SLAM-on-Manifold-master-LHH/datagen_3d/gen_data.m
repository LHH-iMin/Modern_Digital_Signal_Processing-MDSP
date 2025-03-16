function data = gen_data( do_vis )
close all;

% generate data 
if nargin < 1
    do_vis = 1;
end

%addpath('lie_utils/');
config;

% step size
t = 0:1:500;
% t=1:200;
poses = gen_trajectory(t);

newposes=poses;


% plot 3d x y z
%这段代码通过循环遍历所有位姿，根据每个位姿的旋转矩阵和位置信息，
%计算出 x、y、z 轴方向的线段端点，并使用 plot3 函数在三维空间中绘制这些线段，
%最终展示机器人在各个时刻的朝向。这对于理解机器人在环境中的运动和姿态变化非常有帮助。
if do_vis == 1
    figure;
    scatter3( poses.position(1,:), poses.position(2, :), poses.position(3, :), 'r*' );
    hold on;

    %draw axis
    axisl = 5;%缩放变量，旋转矩阵指向线段

    for i = 1:size(poses.euler, 2)
        rotationi = poses.orientation((i-1)*3+1:i*3, :);
        xdir = poses.position(:, i)+axisl*rotationi(:, 1);
        xdir = [poses.position(:, i) xdir];
        ydir = poses.position(:, i)+axisl*rotationi(:, 2);
        ydir = [poses.position(:, i) ydir];
        zdir = poses.position(:, i)+axisl*rotationi(:, 3);
        zdir = [poses.position(:, i) zdir];
        plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red'); hold on;
        plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'green'); hold on;
        plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'blue'); hold on;
    end
end
%% add random points
n_landmarks = N_LANDMARKS;
if ~exist('landmarks')
    % generate new landmarks
    border = [30, 30, 30];
    minpos = min(poses.position');
    maxpos = max(poses.position');
    minlm = minpos - border;
    maxlm = maxpos + border;
    landmarks(:, 1) = minlm(1)+rand(n_landmarks, 1)*(maxlm(1)-minlm(1));
    landmarks(:, 2) = minlm(2)+rand(n_landmarks, 1)*(maxlm(2)-minlm(2));
    landmarks(:, 3) = minlm(3)+rand(n_landmarks, 1)*(maxlm(3)-minlm(3));
    save('landmarks', 'landmarks');
else
    % load landmarks
    load('landmarks');
end

%% draw landmarks
if do_vis == 1
    scatter3( landmarks(:, 1), landmarks(:, 2), landmarks(:, 3), 'k^' ); hold on;
end

%% generate odom生成里程计数据，坐标变换和转向角
odoms = [];
for i = 1:size(poses.position, 2)-1;
    % compute odometry in so(3)
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    rotationi1 = poses.orientation(i*3+1:(i+1)*3, :);
    rotation_diff = rotationi'*rotationi1;
    rotation_diff_so3 = so3_log(rotation_diff);%将相对旋转矩阵转换为三维旋转向量（在SO(3)李代数空间中）
    translation_diff = poses.position(:, i+1)-poses.position(:, i);
    translation_diff = rotationi'*translation_diff;        % change
    
    n1=SIGMA_ODOM*randn(3,1).*rotation_diff_so3;
    n2=SIGMA_ODOM*randn(3,1).*translation_diff;
    
    newposes.orientation(i*3+1:(i+1)*3, :)=newposes.orientation((i-1)*3+1:i*3, :)*so3_exp(n1)*rotation_diff;
    newposes.position(:, i+1)=newposes.position(:, i)+newposes.orientation((i-1)*3+1:i*3, :)*( so3_exp(n1)* translation_diff+jaco_r(-n1)* n2 );
   
    odomi = [translation_diff; rotation_diff_so3]';
    odoms = [odoms; odomi];
end

poses=newposes;




%% generate observations生成观测数据
obsers = {};
obsers_real={};
for i = 1:size(poses.position, 2) % for pose
    posi = poses.position(:, i);
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    zi = rotationi(:, 3)';
    obseri = [];
    obseri_real=[];
    for j = 1:size(landmarks, 1)
        ptjdir = landmarks(j, :) - posi';%机器人与地标之间的坐标差
        ptjangle = dot(ptjdir, zi)/(norm(ptjdir)*norm(zi));
        ptjangle = acos(ptjangle);%机器人到地标的指向
        if ptjangle < MAX_DEGREE && norm(ptjdir) < MAX_RANGE%如果角度在可观测视野角度内，以及距离小于可观测最远距离
            ptjdir = rotationi'*ptjdir';
            obserij = [j, ptjdir'];
            obserij_real=[j, ptjdir'];
            if ADD_NOISE == 1%在理论观测数据上添加噪声，生成真实观测数据
                noise_ob = obserij(2:4)*SIGMA_OBSV.*randn(3, 1)';
                obserij(2:4)=obserij(2:4)+noise_ob;  %change
            end
            obseri = [obseri; obserij];
            obseri_real=[obseri_real; obserij_real   ];
        end
    end
    obsers{i} = obseri;
    obsers_real{i}=obseri_real;
    %每一个元素代表在i时刻的观测数据：N*4维度的矩阵，第一列是地标的编号，2到4列是坐标差乘以旋转矩阵后的结果
end
%title('3D Simulation Data Generator');


%% convert data format
indy = 1;
for i = 1:size(odoms, 1)
    % set observation
    for j = 1:size(obsers{i}, 1)%获得每一时刻的观测数据：观测到地标的个数
        % ztfile的三行才对应一个地标的观测数据
        ztfile(indy, 1) = obsers{i}(j, 2); %带噪声的x观测数据
        ztfile(indy, 2) = 2;%2表示是观测数据，用来和里程计数据作为区分
        ztfile(indy, 3) = obsers{i}(j, 1);%地标的编号
        ztfile(indy, 4) = i-1;%观测的时刻
        ztfile(indy, 5) = obsers_real{i}(j,2);%不带噪声的x观测数据
        
        indy = indy+1;
        ztfile(indy, 1) = obsers{i}(j, 3); %带噪声的y观测数据
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);%地标的编号
        ztfile(indy, 4) = i-1;
        ztfile(indy, 5) = obsers_real{i}(j,3);%不带噪声的y观测数据

        indy = indy+1;
        ztfile(indy, 1) = obsers{i}(j, 4); %带噪声的z观测数据
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);%地标的编号
        ztfile(indy, 4) = i-1;
        ztfile(indy, 5) = obsers_real{i}(j,4);%不带噪声的z观测数据

        indy = indy+1;
    end
    % set odometry观测数据搞完之后是里程计数据，一共六行
    for j = 1:6
        ztfile(indy, 1) = odoms(i, j); 
        ztfile(indy, 2) = 1;%1表示是里程计数据，用来和观测数据作为区分
        ztfile(indy, 3) = i;
        ztfile(indy, 4) = i-1;
        indy = indy+1;
    end  
end

% set observation for the last frame设置最后一帧的观测数据
index = length(obsers);
if size(obsers{index}, 1)==0
    disp("最后一帧没有观测数据")
end
for j = 1:size(obsers{index}, 1)
    ztfile(indy, 1) = obsers{index}(j, 2); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    ztfile(indy, 5) = obsers_real{index}(j,2);
    
    indy = indy+1;
    ztfile(indy, 1) = obsers{index}(j, 3); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    ztfile(indy, 5) = obsers_real{index}(j,3);
 
    indy = indy+1;
    ztfile(indy, 1) = obsers{index}(j, 4); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    ztfile(indy, 5) = obsers_real{index}(j,4);
    indy = indy+1;
end
data.state = ztfile; % state vector观测数据和里程计数据
data.obse_cov = OBSV_NOISE; % observation covariance matrix观测噪声协方差矩阵
data.odom_cov = ODOM_NOISE; % odometry covariance matrix里程计噪声协方差矩阵

data.odom_sigma = SIGMA_ODOM;
data.obsv_sigma = SIGMA_OBSV;

data.landmarks=landmarks;
data.poses=poses;%机器人位置与旋转矩阵
save data data

clearvars -except data;

