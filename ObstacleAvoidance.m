%   ROBOTICS FUNDAMENTALS
%   Code published by "masin" for education and professional portfolio purposes only. Any plagarism will not be tolerated.

clear
close all
clc
%parameters of links
L1 = 6; L2 = 15; L3 = 15; L4 = 3.5; L5 = 6.5;
figure (1)
set(1,'position',[0 0 800 600])
PlotCube([8*sqrt(3)-12 8*sqrt(3)-12 8*sqrt(3)-12],[-11  -6  0],1,[0.5 0.5 0.5]);
axis equal
axis([-25 20 -10 20 0 30])
grid on
hold on

% time interval is 0.2s
ti = 0.2;
% acceleration time = 0.5s
ta = 0.5;
% constant time = 3.5s
tc = 3.5;
% acceleration = 5 cm/s^2
a = 5;
% constant velocity = 2.5 cm/s
v = 2.5;

% TOP
for t = 0:ti:ta
    py = 5;
    px = a*t^2/2 - 15;
    pz = 0;
    DrawLine(px,py,pz);
end

for t = 0:ti:tc
    py = 5;
    px = v*t - 14.375;
    pz = 0;
    DrawLine(px,py,pz);
end

for t = 0:ti:ta    
    py = 5;
    px = v*t - a*t^2/2 - 5.625;
    pz = 0;   
    DrawLine(px,py,pz);
end

% RIGHT
for t = 0:ti:ta
    py = -a*t^2/2 + 5;
    px = -5;
    pz = 0;
    DrawLine(px,py,pz);
end

for t = 0:ti:tc
    py = -v*t + 4.375;
    px = -5;
    pz = 0;   
    DrawLine(px,py,pz);
end

for t = 0:ti:ta
    py = -v*t + a*t^2/2 -4.375;
    px = -5;
    pz = 0;   
    DrawLine(px,py,pz);
end

% BOTTOM
q = [];
xset = [];
yset = [];
zset = [];

for t = 0:ti:ta
    py = -5;
    px = -a*t^2/2 - 5;
    pz = 0;
    DrawLine(px,py,pz);
end
for t = 0:ti:((2.5-0.625)/v)
    py = -5;
    px = -v*t - 5.625;
    pz = 0;  
    DrawLine(px,py,pz);
end
for t = 0.1:ti:(sqrt(3)*pi/6/v)    
    r = sqrt(3)/2;
    w = v/r;    
    theta = -0.5*pi - w*t;   
    px = r*cos(theta) - 7.5;
    py = -5;
    pz = r*sin(theta) + sqrt(3)/2;    
    b = AvoidLine(px,py,pz);   
    q = [q;b];
    xset = [xset, px];
    yset = [yset, py];
    zset = [zset, pz];
end
for t = 0.1:ti:((12.5-6*sqrt(3))/v)    
    px = -0.5*v*t - 8.25;
    py = -5;
    pz = -2*px + sqrt(3)/4 - 16.5;    
    b = AvoidLine(px,py,pz);    
    q = [q;b];
    xset = [xset, px];
    yset = [yset, py];
    zset = [zset, pz];
end
for t = 0.15:ti:((12 - 6*sqrt(3))*pi/3/v)   
    r = 6 - 3*sqrt(3);
    w = v/r;   
    theta = 1/6*pi + w*t;   
    px = r*cos(theta) - 10;
    py = -5;
    pz = r*sin(theta) + 8*sqrt(3) -12;    
    b = AvoidLine(px,py,pz);   
    q = [q;b];
    xset = [xset, px];
    yset = [yset, py];
    zset = [zset, pz];
end
for t = 0.1:ti:((12.5-6*sqrt(3))/v)
    px = -0.5*v*t - 5.5-3*sqrt(3);
    py = -5;
    pz = 2*px + 12.5*sqrt(3) + 2;   
    b = AvoidLine(px,py,pz);   
    q = [q;b];
    xset = [xset, px];
    yset = [yset, py];
    zset = [zset, pz];
end
for t = 0.1:ti:(sqrt(3)*pi/6/v)    
    r = sqrt(3)/2;
    w = v/r;    
    theta = -1/6*pi - w*t;    
    px = r*cos(theta) - 12.5;
    py = -5;
    pz = r*sin(theta) + sqrt(3)/2;    
    b = AvoidLine(px,py,pz);
    q = [q;b];
    xset = [xset, px];
    yset = [yset, py];
    zset = [zset, pz];
end
for t = 0.1:ti:((2.5-0.625)/v)
    py = -5;
    px = -v*t - 12.5;
    pz = 0;    
    DrawLine(px,py,pz);
end
for t = 0:ti:ta
    py = -5;
    px = -v*t + a*t^2/2 - 14.375;
    pz = 0;    
    DrawLine(px,py,pz);
end

% LEFT
for t = 0:ti:ta
    py = a*t^2/2 - 5;
    px = -15;
    pz = 0;
    DrawLine(px,py,pz);
end
for t = 0:ti:tc
    py = v*t -4.375;
    px = -15;
    pz = 0;
    DrawLine(px,py,pz);
end
for t = 0:ti:ta
    py = v*t - a*t^2/2 + 4.375;
    px = -15;
    pz = 0;
    DrawLine(px,py,pz);
end
