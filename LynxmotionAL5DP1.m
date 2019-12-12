%   ROBOTICS FUNDAMENTALS
%   Liam Weight 2019
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
% Distance between joints in (mm)
l1 = 50;    l2 = 120;   l3 = 130;   l4 = 40;    l5 = 20;
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
plotWorkspace = 1;
if plotWorkspace == 1
    %Define the "th" orientations to "iterate" (i) through, in degrees 
    ith2 = 0; ith3 = 0; ith4 = -90; ith5 = 0;  ithe = 0;
    x = []; y = []; z = [];
    figure
    clf
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    hold on
    for ith1 = 0:10:180
        for ith2 = 0:15:180
            for ith3 = -150:15:0
                for ith4 = -180:15:0
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

%% Path Planning

xEff = [130,130,24,-24,-130,-130];
yEff = [0,24,130,130,24,0];
zEff = [0,100,142,140,100,0];
pEff = [-90,0,30,20,0,-90];

xePathLine = [ linspace(xEff(1),xEff(2),20), linspace(xEff(2),xEff(3),20), linspace(xEff(3),xEff(4),20), linspace(xEff(4),xEff(5),20), linspace(xEff(5),xEff(6),20)];
yePathLine = [ linspace(yEff(1),yEff(2),20), linspace(yEff(2),yEff(3),20), linspace(yEff(3),yEff(4),20), linspace(yEff(4),yEff(5),20), linspace(yEff(5),yEff(6),20)];
zePathLine = [ linspace(zEff(1),zEff(2),20), linspace(zEff(2),zEff(3),20), linspace(zEff(3),zEff(4),20), linspace(zEff(4),zEff(5),20), linspace(zEff(5),zEff(6),20)];
pLine = [ linspace(pEff(1),pEff(2),20),...
    linspace(pEff(2),pEff(3),20),...
    linspace(pEff(3),pEff(4),20),...
    linspace(pEff(4),pEff(5),20),...
    linspace(pEff(5),pEff(6),20)];

%%% Free Path
xPath = @(step) 2*10^(-18)*step^4 + 0.0007*step^3 - 0.1027*step^2 + 0.8244*step + 132.95;
yPath = @(step) yStep(step);
zPath = @(step) -0.0587*step^2 + 5.9227*step - 4.1244;

%%% Free Path Joint space interpolation
th1FP = [0,10,80,100,170,180];
th2FP = [67,110,109,113,110,67];
th3FP = [-117,-139,-132	,-132,-139,-117];
th4FP = [-40,29,53,38,29,-40];

th1FMP = [ linspace(th1FP(1),th1FP(2),10), linspace(th1FP(2),th1FP(3),10), linspace(th1FP(3),th1FP(4),10), linspace(th1FP(4),th1FP(5),10), linspace(th1FP(5),th1FP(6),10) ];
th2FMP = [ linspace(th2FP(1),th2FP(2),10), linspace(th2FP(2),th2FP(3),10), linspace(th2FP(3),th2FP(4),10), linspace(th2FP(4),th2FP(5),10), linspace(th2FP(5),th2FP(6),10) ];
th3FMP = [ linspace(th3FP(1),th3FP(2),10), linspace(th3FP(2),th3FP(3),10), linspace(th3FP(3),th3FP(4),10), linspace(th3FP(4),th3FP(5),10), linspace(th3FP(5),th3FP(6),10) ];
th4FMP = [ linspace(th4FP(1),th4FP(2),10), linspace(th4FP(2),th4FP(3),10), linspace(th4FP(3),th4FP(4),10), linspace(th4FP(4),th4FP(5),10), linspace(th4FP(5),th4FP(6),10) ];

%% Obstacle Avoidance:
% Defining obstacle as a cylinder with infinte z length.
obD = 23;
obx = 80;
obs_yc = 65;
obstacleX = zeros(obD);
obstacleY = zeros(obD);

% populate coordinates of obstacle
indexX = 1;
indexY = 1;
for x = -11:11
    for yy = -11:11
        y = -(yy);
        r = sqrt((x)^2+(y)^2); % w.r.t obstacle centre at (75,75)
        if r <= 11 % radius
            obstacleX(indexX) = 80+x;
            obstacleY(indexY) = 65+y;
        end
        indexX = indexX + 1;
        indexY = indexY + 1;
    end
end
obsPathX = [];
obsPathY = [];
xpAgg = [];
ypAgg = [];
for p = 1:numel(xePathLine)
    xp = xePathLine(p);
    yp = yePathLine(p);
    hipX = ismember(round(xp),obstacleX);
    membershipX = ismember(round(xp),obstacleX);
    membershipY = ismember(round(yp),obstacleY);
    
    % if both values are in the cylinder, then update to a new point
    if membershipX == 1 && membershipY == 1
        obstructed = 1;
        coss = xp/hyp(xp,yp);
        sinn = yp/hyp(xp,yp); 
        while obstructed
            xp = xp - 1;%*coss;
            yp = yp - 1;%*sinn;
            xpAgg(end+1) = xp;
            ypAgg(end+1) = yp;
            membershipX = ismember(round(xp),obstacleX);
            membershipY = ismember(round(yp),obstacleY);
            if membershipX == 0 || membershipY == 0
                disp('xpBreak: '),disp(xp)
                disp('ypBreak: '),disp(yp)
                break
            end
        end
    end
 
    obsPathX(end+1) = xp;
    obsPathY(end+1) = yp;
    
end



%% Inverse Kinematics

% Note, Theta 5 has been ignored since it does not change the coordinates
% of EE but only orientation, which is not required in this application

% initialise for safety
xe = -130;
ye = 0;
ze = 0;
pitch = deg2rad(-90);

clc

% define the coordinates, either constants or variables defined by a
% function


% xe = T0E(1,4);
% ye = T0E(2,4);
% ze = T0E(3,4);



% iTheta is the angle
% iTheta = 0;

% xe = (300-iTheta*2)*cos(iTheta);
% ye = (300-iTheta*2)*sin(iTheta);
% ze = (300-iTheta*5)*sin(iTheta);

signz = 1;
signx = 1;
signy = 1;

xfeAgg = [];
yfeAgg = [];
zfeAgg = [];

% pitchPath is the pitch angle of the EE, created as "linspace"
% pitchPath = 0:pi/800:pi/2;
% bIndex = 0; % initialise the index for accessing pitchPath


[m,n] = size(xePathLine);


% use these to toggle between different path modes
chooseLinePath = 1; %make n = 100 down below
chooseFreePath = 0; %make n = 100 down below
jointSpaceFreeMotion = 0; %make n = 50 down below
obstacleAvoidancePath = 1; %make n = 100 down below
%                               ..
%                               ..
for iTheta = 1:100 % < ............  n is here
    %specify end effector location
    % update EE coordinates for next iTheta (next step). "iTheta:inverseTheta"
    
    if chooseLinePath == 1
        xe = xePathLine(iTheta);
        ye = yePathLine(iTheta);
        ze = zePathLine(iTheta);
    end
    
    if chooseFreePath == 1
        xe = xPath(iTheta);
        ye = yPath(iTheta);
        ze = zPath(iTheta);
    end
    
    if obstacleAvoidancePath == 1
        xe = obsPathX(iTheta);
        ye = obsPathY(iTheta);
        ze = zPath(iTheta);
    end
    
    % xe = [130,130,24,-24,-130,-130];
    % ye = [0,24,130,130,24,0];
    % ze = [0,100,142,140,100,0];
    % BLinDeg = [-90,0,30,20,0,-90];
    
    % xe = -130;
    % ye = 0;
    % ze = 0;
    % pitch = degtorad(-90);
    
    pitch = deg2rad(pLine(iTheta));
    
    
    % specify pitch angle
    %pitch = sym('pi')/2 - abs(pitch(T0E));%*cos(roll(T0E));
    
    % increment pitchPath access index to get next pitch angle for EE
    % bIndex = bIndex+1;
    %pitch = ;%degtorad(-90);%pitchPath(bIndex);
    
    
    % re: euclidean distance of EE in the X,Y plane from the base of the robot
    % or {0}
    
    if jointSpaceFreeMotion == 0
        re = sqrt(xe^2+ye^2);
        
        %r4 : euclidean distance of {4}, extracted from re and the pitch angle of
        %the EE, which defines the angle theta4
        r4 = re - abs((l4+l5)*cos(pitch));
        
        % sign: defines in what orientation/quadrant the end efector is with respect to {4}
        % this has been done after empirical testing. and due to the shift in z
        % coordinates w.r.t the base of the robot.
        sign = cos(roll(T0E));
        
        % z4: z coordinate of {4}
        z4 = ze - ((l4+l5)*sin(pitch)) - l1;  % was after first - sign*abs
        
        
        % IK as defined in the report.
        %
        IK_cos_3 = (r4^2 + z4^2 - l2^2 - l3^2) / (2*l2*l3);
        IK_sin_3 = [real(sqrt(1-IK_cos_3^2)) , -real(sqrt(1-IK_cos_3^2))];
        
        
        IK_th1 = atan2(ye,xe);
        
        IK_th3_A = atan2(IK_sin_3(1),IK_cos_3);
        IK_th3_B = atan2(IK_sin_3(2),IK_cos_3);
        
        IK_th2_A = atan2(z4,r4) + atan2(abs(l3*sin(IK_th3_A)),l2+l3*cos(IK_th3_A));
        IK_th2_B = atan2(z4,r4) + atan2(abs(l3*sin(IK_th3_B)),l2+l3*cos(IK_th3_B));
        
        IK_th4_A = sign*pitch - IK_th3_A - IK_th2_A ;
        IK_th4_B = sign*pitch - IK_th3_B - IK_th2_B ;
        
        IK_th1_deg = vpa(rad2deg(IK_th1));
        IK_th2_deg_A = vpa(rad2deg(IK_th2_A));
        IK_th2_deg_B = vpa(rad2deg(IK_th2_B));
        
        IK_th3_deg_A = vpa(rad2deg(IK_th3_A));
        IK_th3_deg_B = vpa(rad2deg(IK_th3_B));
        
        IK_th4_deg_A = vpa(rad2deg(IK_th4_A));
        IK_th4_deg_B = vpa(rad2deg(IK_th4_B));
        
        % IK works for extreme limits (simple reaching boundary) tests
        
        disp('IK_th1_deg'), disp(IK_th1_deg)
        disp('IK_th2_deg_AB'), disp([IK_th2_deg_A,IK_th2_deg_B])
        disp('IK_th3_deg_AB'), disp([IK_th3_deg_A,IK_th3_deg_B])
        disp('IK_th4_degAB'), disp([IK_th4_deg_A,IK_th4_deg_B])
        
    end
    % Getting the location of each joint from the inverskinematic equations in
    % order to plot the robot.
    
    
    if jointSpaceFreeMotion == 1
        
        IK_th1   = deg2rad(th1FMP(iTheta));
        IK_th2_A = deg2rad(th2FMP(iTheta));
        IK_th3_B = deg2rad(th3FMP(iTheta));
        IK_th4_B = deg2rad(th4FMP(iTheta));
        
        pause(0.1)
    end
    
    rf2 = l2*cos(IK_th2_A);
    xf2 = rf2*cos(IK_th1);
    yf2 = rf2*sin(IK_th1);
    zf2 = l2*sin(IK_th2_A)+l1;
    
    rf3 = rf2 + l3*cos(IK_th2_A+IK_th3_B);
    xf3 = rf3*cos(IK_th1);
    yf3 = rf3*sin(IK_th1);
    zf3 = zf2 + l3*sin(IK_th2_A+IK_th3_B);
    
    rf4 = rf3 + l4*cos(IK_th2_A+IK_th3_B+IK_th4_B);
    xf4 = rf4*cos(IK_th1);
    yf4 = rf4*sin(IK_th1);
    zf4 = zf3 + l4*sin(IK_th2_A+IK_th3_B+IK_th4_B);
    
    rfe = rf4 + l5*cos(IK_th2_A+IK_th3_B+IK_th4_B);
    xfe = rfe*cos(IK_th1);
    yfe = rfe*sin(IK_th1);
    zfe = zf4 + l5*sin(IK_th2_A+IK_th3_B+IK_th4_B);
    
    % the Agg_regate stores all values of the end effetors location in order to
    % plot the path which the EE has traversed
    xfeAgg(end+1) = xfe;
    yfeAgg(end+1) = yfe;
    zfeAgg(end+1) = zfe;
    
    
    % plotting
    plot3([0,0,xf2,xf3,xf4,xfe],[0,0,yf2,yf3,yf4,yfe],[0,l1,zf2,zf3,zf4,zfe])
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    hold on
    grid on
    % circle(0,0,80)
    % circle(0,0,300)
    plot3(xfeAgg,yfeAgg,zfeAgg);
    scatter3([0,0,xf2,xf3,xf4,xfe],[0,0,yf2,yf3,yf4,yfe],[0,l1,zf2,zf3,zf4,zfe])
    
    
    
    % the circles plotted are merely to visualise the movement of the robot in
    % the x,y plane to better understand and avoid problems with 3D visuals.
    circle(0,0,double(rf2))
    circle(0,0,double(rf3))
    circle(0,0,double(rf4))
    
    circle(0,0,double(rfe))
    
    
    % setting the limit of the axes, so the figure is not jumpy with every plot
    xlim([-320 320])
    ylim([-320 320])
    zlim([-50 320])
    
    
    % hold off;
    
    % pause in order to visualise
    pause(0.1)
    
    % xe = 300*cos(iTheta);
    % ye = 300*sin(iTheta);
    % ze = (100-iTheta*5)*sin(iTheta) + 100;
end
