%   ROBOTICS FUNDAMENTALS
%   Code published by "masin" for education and professional portfolio purposes only. Any plagarism will not be tolerated.
%   Part 1
close all
clear
clc

%% Functions
pitch = @(T) atan2(-T(3,1),sqrt((T(3,2))^2+(T(3,3))^2));
yaw = @(T) atan2(T(2,1),T(1,1));
roll = @(T) atan2(T(3,2),T(3,3));
hyp = @(x,y) sqrt(x^2 + y^2);

%%
% Distance between joints in (cm)
l1 = 6;    l2 = 15;   l3 = 15;   l4 = 3.5;    l5 = 6.5;
syms th1 th2 th3 th4 th5

% DH Parameters - th6 not required as does not effect location of effector,
% only orientation. rthi is reference so as to use different variables for
% different purposes throughout the code.
a0 = 0;  al0 = 0;            d1 = l1;  th1 = deg2rad(15);   rth1 = th1;
a1 = 0;  al1 = sym('pi')/2;  d2 = 0;   th2 = deg2rad(60);   rth2 = th2;
a2 = l2; al2 = 0;            d3 = 0;   th3 = deg2rad(-30);  rth3 = th3;
a3 = l3; al3 = 0;            d4 = 0;   th4 = deg2rad(-30);  rth4 = th4 - sym('pi');
a4 = 0;  al4 = -sym('pi')/2; d5 = l4;  th5 = 0;             rth5 = th5;
ae = 0;  ale = 0;            de = l5;                       rthe = 0;

% Defining the transformation function that takes in DH parameters and
% returns the HT matrix for the specified reference frames (two frames).
TransMat = @(aN_1,alN_1,dN,thN) [cos(thN)              -sin(thN)               0               aN_1;...
    sin(thN)*cos(alN_1)    cos(thN)*cos(alN_1)   -sin(alN_1)     -dN*sin(alN_1);...
    sin(thN)*sin(alN_1)    cos(thN)*sin(alN_1)    cos(alN_1)      dN*cos(alN_1);...
    0                      0                      0               1];

%% Finding the Homogenous Transformation matrices for each joint

%Transformation Matrix to orient the robot in relation to the "global" axis.
%  TRM = [1 0 0 0;...
%         0 1 0 0;...
%         0 0 1 0;...
%         0 0 0 1];

T01 = TransMat(a0,al0,d1,rth1);
T12 = TransMat(a1,al1,d2,rth2);
T23 = TransMat(a2,al2,d3,rth3);
T34 = TransMat(a3,al3,d4,rth4);
T45 = TransMat(a4,al4,d5,rth5);
T5E = TransMat(ae,ale,de,rthe);

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T0E = T05*T5E;
% T0E = T01*T12*T23*T34*T45*T5E;
% T15 = inv(T01)*T0E;

%% Plotting the workspace of the robot.
%Set to 0 so as to move onto the trajectories and inverse kinematics.
%Set to 1 to plot workspace.
simWS = 0;
if simWS == 1
    %Define the "th" orientations to "iterate" (i) through, in degrees 
    ith2 = 0; ith3 = 0; ith4 = -90; ith5 = 0;  ithe = 0;
    x = []; y = []; z = [];
    figure
    clf
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    hold on
    for ith1 = 0:20:180
        for ith2 = 0:20:180
            for ith3 = -150:30:0
                for ith4 = -180:20:0
                    T01 = TransMat(a0,al0,d1,deg2rad(ith1));
                    T12 = TransMat(a1,al1,d2,deg2rad(ith2));
                    T23 = TransMat(a2,al2,d3,deg2rad(ith3));
                    T34 = TransMat(a3,al3,d4,deg2rad(ith4));
                    T45 = TransMat(a4,al4,d5,deg2rad(ith5));
                    T5E = TransMat(ae,ale,de,deg2rad(ithe));
                    T0E = T01*T12*T23*T34*T45*T5E;
                    xtemp = T0E(1,4);
                    ytemp = T0E(2,4);
                    ztemp = T0E(3,4);
                    x(end+1) = xtemp;
                    y(end+1) = ytemp;
                    z(end+1) = ztemp;
                    scatter3(xtemp,ytemp,ztemp);
                    pause(0.1)
                end
            end
        end
    end
    scatter3(x,y,z);
    xlabel('X')
    ylabel('Y')
    zlabel('Z')  
end
