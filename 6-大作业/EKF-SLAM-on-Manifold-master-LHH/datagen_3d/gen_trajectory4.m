function [ Vehicle ] = gen_trajectory4( t )
% generate trajectory given step t
% t     - step
     radius = 10;   
    n=50;
    Vehicle.position=zeros(3,size(t,2));
    Vehicle.euler=zeros(3,size(t,2));
    Vehicle.position(1:3,1:n) = radius* [ 5.1*cos(0*t(1:n)+5); 4.03*sin(0*t(1:n)+6); 2.1*sin(0*t(1:n)+8) ];
    Vehicle.position(1:3,n+1:end)=[5.1*cos(0*t(n+1:end)+5)+n; 4.03*sin(0*t(n+1:end)+6)+40; 2.1*sin(0*t(n+1:end)+8)+30];
%     Vehicle.euler(1:3,1:n) =  [0*t(1:n)+3; -0*t(1:n)+2; 0*t(1:n)+1];
%     Vehicle.euler(1:3,n+1:end) =  [0*t(n+1:end)+3+0.5; -0*t(n+1:end)+2+0.5; 0*t(n+1:end)+1+0.5];
    Vehicle.euler=  [0*t+3; -0*t+2; 0*t+1];


    
    
    
    
    % generate rotation matrix
    for i = 1:size(Vehicle.euler, 2)
        Vehicle.orientation((i-1)*3+1:i*3, 1:3) = euler2rotation_matrix(Vehicle.euler(:, i));
    end
end