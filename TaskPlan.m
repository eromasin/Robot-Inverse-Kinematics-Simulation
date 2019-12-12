%   ROBOTICS FUNDAMENTALS
%   Liam Weight 2019

clear
close all
clc
% LINK PARAMETERS
L1 = 6; L2 = 15; L3 = 15; L4 = 3.5; L5 = 6.5; 
figure (1)
set(1,'position',[0 0 800 600])

% TIME ITERATION
ti = 0.2;
% ACCELERATION TIME
ta = 0.5;
% TIME CONSTANT
tc = 3.5;
% ACCELERATION
a = 5;
% VELOCITY
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
for t = 0:ti:ta
    py = -5;
    px = -a*t^2/2 - 5;
    pz = 0;
    DrawLine(px,py,pz);
end
for t = 0:ti:tc
    py = -5;
    px = -v*t - 5.625;
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