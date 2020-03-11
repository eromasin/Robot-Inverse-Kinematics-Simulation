%   ROBOTICS FUNDAMENTALS
%   Code published by "masin" for education and professional portfolio purposes only. Any plagarism will not be tolerated.
function a = AvoidLine(px, py, pz)
    L1 = 6; L2 = 15; L3 = 15; L4 = 3.5; L5 = 6.5;     
    cw24 = -1;
    sw24 = 0;
    theta1 = atan2(py,px);
    if theta1<-pi/2
        theta1 = theta1 + pi;
    elseif theta1>pi/2
        theta1 = theta1 - pi;
    end
    cw1 = cos(theta1);
    sw1 = sin(theta1);
    ax=-cw1*sw24;
    ay=-sw1*sw24;
    az=cw24;
    D=(((pz-az*(L4+L5)-L1)^2+(px-ax*(L4+L5))^2+(py-ay*(L4+L5))^2)-L2^2-L3^2)/(2*L2*L3);
    theta3=atan2(sqrt(1-D^2),D);
    a = L2 + L3*D;
    d = L3*sqrt(1-D^2);
    K = cos(theta1)*px + sin(theta1)*py + sw24*(L4+L5);
    theta2 = [0;0];
    theta2(1) = atan2(a,d) - atan2(K,sqrt(a^2+d^2-K^2));
    theta2(2) = atan2(a,d) - atan2(K,-sqrt(a^2+d^2-K^2));
    theta234 = atan2(sw24,cw24);
    if theta234<0
        theta234 = theta234+2*pi;
    end
    theta4 = [0;0];
    theta4(1) = theta234 - theta2(1) - theta3;
    theta4(2) = theta234 - theta2(2) - theta3;
    a = [theta1*180/pi, theta2(1) *180/pi, theta3*180/pi, theta4(1)*180/pi, 0];
    q1 = theta1;
    q2 = theta2(1);
    q3 = theta3;
    q4 = theta4(1);
    c1 = cos(q1);  c2 = cos(q2);
    c23 = cos(q2 + q3);  c24 = cos(q2 + q3 + q4);
    s1 = sin(q1);  s2 = sin(q2);
    s23 = sin(q2 + q3);  s24 = sin(q2 + q3 + q4);
    x0 = 0;    x1 = 0;
    x2 = L2 * c2 * c1;
    x3 = x2 + L3 * c23 * c1;
    x4 = x3 - L4 * s24 * c1;
    xe = x4 - L5 * s24 * c1;
    y0 = 0;    y1 = 0;
    y2 = L2 * c2 * s1;
    y3 = y2 + L3 * c23 * s1;
    y4 = y3 - L4 * s24 * s1;
    ye = y4 - L5 * s24 * s1;
    z0 = 0;    z1 = L1;
    z2 = L1 + L2 * s2;
    z3 = z2 + L3 * s23;
    z4 = z3 + L4 * c24;
    ze = z4 + L5 * c24;
    xx0 = [x0 x1 x2 x3 x4 xe];
    yy0 = [y0 y1 y2 y3 y4 ye];
    zz0 = [z0 z1 z2 z3 z4 ze];
    figure(1)
    p = plot3(xx0, yy0, zz0, 'ko-','Linewidth',2);
    xlabel('x/cm'); 
    ylabel('y/cm');
    zlabel('z/cm');
    axis([-25 10 -10 20 0 22])
    hold on 
    grid on
    pause(0.1)
    set(p,'visible','off')
    plot3(xe,ye,ze, '.r')
    axis equal
    axis([-25 10 -10 20 0 22])
    hold on
    grid on
end
