%   ROBOTICS FUNDAMENTALS
%   Code published by "masin" for education and professional portfolio purposes only. Any plagarism will not be tolerated.
clc
close all
clear 
figure (1)
set(1,'position',[0 0 800 600])
L1 = 6; L2 = 15; L3 = 15; L4 = 3.5; L5 = 6.5; 
tf = 3; 
d = 0.2;  
px = zeros(1, tf/d+1);
py = zeros(1, tf/d+1);
pz = zeros(1, tf/d+1);
i = 0;
for t = 0:d:tf     
    i = i + 1;    
    theta10 = -0.3218; theta20 = 1.8978; theta30 = 1.992; theta40 = -0.7482;
    theta1f = -1.1071; theta2f = 1.6342; theta3f = 2.3277; theta4f = -0.8203; 
    q1 = theta10 + 3*(theta1f-theta10)/(tf^2)*(t^2) - 2*(theta1f-theta10)/(tf^3)*(t^3);
    q2 = theta20 + 3*(theta2f-theta20)/(tf^2)*(t^2) - 2*(theta2f-theta20)/(tf^3)*(t^3);
    q3 = theta30 + 3*(theta3f-theta30)/(tf^2)*(t^2) - 2*(theta3f-theta30)/(tf^3)*(t^3);
    q4 = theta40 + 3*(theta4f-theta40)/(tf^2)*(t^2) - 2*(theta4f-theta40)/(tf^3)*(t^3);
    c1 = cos(q1); c2 = cos(q2); c3 = cos(q3); c4 = cos(q4);
    c23 = cos(q2 + q3); c24 = cos(q2 + q3 + q4);
    s1 = sin(q1); s2 = sin(q2); s3 = sin(q3); s4 = sin(q4);
    s23 = sin(q2 + q3);  s24 = sin(q2 + q3 + q4);
    x0 = 0;
    x1 = 0;
    x2 = L2 * c2 * c1;
    x3 = x2 + L3 * c23 * c1;
    x4 = x3 - L4 * s24 * c1;
    xe = x4 - L5 * s24 * c1;
    px(i) = xe;
    y0 = 0;
    y1 = 0;
    y2 = L2 * c2 * s1;
    y3 = y2 + L3 * c23 * s1;
    y4 = y3 - L4 * s24 * s1;
    ye = y4 - L5 * s24 * s1;
    py(i) = ye;
    z0 = 0;
    z1 = L1;
    z2 = L1 + L2 * s2;
    z3 = z2 + L3 * s23;
    z4 = z3 + L4 * c24;
    ze = z4 + L5 * c24;
    pz(i) = ze;
    xx0 = [x0 x1 x2 x3 x4 xe];
    yy0 = [y0 y1 y2 y3 y4 ye];
    zz0 = [z0 z1 z2 z3 z4 ze];
    figure(1)
    p = plot3(xx0, yy0, zz0, 'bo-','Linewidth',2);
    xlabel('x/cm'); 
    ylabel('y/cm');
    zlabel('z/cm');
    axis equal
    axis([-25 10 -10 20 0 22])
    hold on
    grid on
    pause(0.2)
    set(p,'visible','off')
    plot3(xe, ye, ze, '.')
    axis equal
    axis([-25 10 -10 20 0 22])
    hold on
    grid on

end
