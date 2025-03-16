function [ Vehicle ] = gen_trajectory( t)
% 定义函数gen_trajectory，输入参数t（时间），返回Vehicle结构体，包含机器人轨迹信息

    % 定义振幅
    Ax = 5;   
    Ay = 4;
    Az = 2;
    % 定义频率
    wx = 0.3;   
    wy = 0.2;
    wz = 0.4;

    % 定义相位差
    phix = 0;   
    phiy = 0;
    phiz = 1;

    % 定义缩放半径
    radius = 15;   

    % 计算机器人位置坐标（3D），按照三维Lissajous曲线公式计算
    Vehicle.position = radius* [ Ax*sin(wx*t + phix); Ay*sin(wy*t + phiy); Az*sin(wz*t + phiz) ];
    % 结果存储为3×N矩阵（N为时间点数），每列对应一个时间点的坐标

    Vehicle.euler = [0.5*t+2; -0.3*t; 0.4*t];
    % 计算欧拉角（绕各轴的旋转角）：
    % 第一行：绕X轴角度随时间线性变化，斜率0.5 rad/单位时间，初始角2 rad
    % 第二行：绕Y轴角度，斜率-0.3 rad/单位时间
    % 第三行：绕Z轴角度，斜率0.4 rad/单位时间
    % 结果存储为3×N矩阵，每列对应一个时间点的欧拉角

    % 生成旋转矩阵
    for i = 1:size(Vehicle.euler, 2)
        % 遍历每个时间点（按列循环）
        Vehicle.orientation((i-1)*3+1:i*3, 1:3) = euler2rotation_matrix(Vehicle.euler(:, i));
        % 将第i个时间点的欧拉角转换为3x3旋转矩阵
        % 存储到orientation的连续3行中：
        % 第i个矩阵存储在行索引 (i-1)*3+1 到 i*3，列1-3
        % 例如，i=1时存1-3行，i=2时存4-6行，依此类推
    end
end