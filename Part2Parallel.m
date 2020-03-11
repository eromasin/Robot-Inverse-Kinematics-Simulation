% Code published by "masin" for education and professional portfolio purposes only. Any plagarism will not be tolerated.

clear all
clc
close all
loop=1;
while loop<=2
    


%Design parameters of planar parallel robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sa=170;
L=130;
rPlatform=130;
rBase=290;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Assigning cartesian input parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if loop==1
fprintf('Enter cartesian input parameters for position 1: \n')    
end
if loop==2
fprintf('\n Enter cartesian input par2ameters for position 2: \n')    
end
commandwindow;
Xc=input("Xc: ");
Yc=input("Yc: ");
XcYc=[Xc; Yc];
alpha=input("alpha: ");
fprintf('####################################################### \n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Angular joint positions of the planar parallel robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PB1=pi/2;
PB2=pi+pi/6;
PB3=2*pi-pi/6;
PP1=pi/2;
PP2=pi+pi/6;
PP3=2*pi-pi/6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%calculating the distance vectors from centre of platform to the corners of
%the platform
CPP=zeros(2,3);

for i=1:3
    CPP(1,i)=-rPlatform*cos((alpha+30+120*(i-1))*(pi()/180))+XcYc(1);
    CPP(2,i)=-rPlatform*sin((alpha+30+120*(i-1))*(pi()/180))+XcYc(2);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%calculating the distance between the corners of the base and the centre of
%the base
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PBB=zeros(2,3);

for i=1:3
    PBB(1,i)=-rBase*cos((210+120*(i-1))*(pi()/180));
    PBB(2,i)=-rBase*sin((210+120*(i-1))*(pi()/180));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%calculating the distance between the corners of the base and the corners
%of the platform
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PBPP=zeros(2,3);

for i=1:3
    PBPP(1,i)=PBB(1,i)+CPP(1,i);
    PBPP(2,i)=PBB(2,i)+CPP(2,i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%calculating the theta values and psi values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

e1=zeros(1,3);
e2=zeros(1,3);
e3=zeros(1,3);
t=zeros(1,3);
Theta=zeros(1,3);
Thetadeg=zeros(1,3);
s_psi = zeros(1,3);
psi=zeros(1,3);

for i=1:3
    e1(i)=-2*PBPP(2,i)*Sa;
    e2(i)=-2*PBPP(1,i)*Sa;
    e3(i)=(PBPP(1,i))^2+(PBPP(2,i))^2+Sa^2-L^2;
    t(i)=(-e1(i)+sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
    Theta(i)=2*atan(t(i));
    Thetadeg(i)=rad2deg(2*atan(t(i)));
    
    c_psi = ( PBPP(1,i) - Sa*cosd(Theta(i)) ) / L;
    s_psi = ( PBPP(2,i) - Sa*sind(Theta(i)) ) / L;

    psi(i) = atan2d(s_psi,c_psi);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculating joint positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Joint=zeros(2,3);

for i=1:3
    Joint(1,i)=-PBB(1,i)+Sa*cos(Theta(i));
    Joint(2,i)=-PBB(2,i)+Sa*sin(Theta(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%assigning values of the base and platform and each individual joint on each link
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

base=[-PBB(1,:) -PBB(1,1);-PBB(2,:) -PBB(2,1)];
plat=[CPP(1,:) CPP(1,1);CPP(2,:) CPP(2,1)];
links1=[-PBB(1,1) Joint(1,1) CPP(1,1);-PBB(2,1) Joint(2,1) CPP(2,1)];
links2=[-PBB(1,2) Joint(1,2) CPP(1,2);-PBB(2,2) Joint(2,2) CPP(2,2)];
links3=[-PBB(1,3) Joint(1,3) CPP(1,3);-PBB(2,3) Joint(2,3) CPP(2,3)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%Plot kinematic model in position 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if loop==1
figure(1);
hold on;
set(gcf,'Position',[300 200 700 400])
title("Simulated Robot Motion with cartesian position Xc(mm):"+Xc+" Yc(mm):"+Yc+" alpha(deg):"+alpha); xlabel('x (mm)') ; ylabel('y (mm)'); zlabel('z (mm)');
plot(XcYc(1),XcYc(2),'m+');

plot(0,0,'blue+');
plot(plat(1,:),plat(2,:), 'Color', 'm');
plot(base(1,:),base(2,:), 'Color', 'b');
plot(links1(1,:),links1(2,:), 'Color', 'g');
plot(links2(1,:),links2(2,:), 'Color', 'g');
plot(links3(1,:),links3(2,:), 'Color', 'g');
hold off;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Plot kinematic model in position 2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if loop==2
figure(2);
hold on;
set(gcf,'Position',[1100 200 700 400])
title("Simulated Robot Motion with cartesian position Xc(mm):"+Xc+" Yc(mm):"+Yc+" alpha(deg):"+alpha); xlabel('x (mm)') ; ylabel('y (mm)'); zlabel('z (mm)');plot(XcYc(1),XcYc(2),'m+');

plot(0,0,'blue+');
plot(plat(1,:),plat(2,:), 'Color', 'm');
plot(base(1,:),base(2,:), 'Color', 'b');
plot(links1(1,:),links1(2,:), 'Color', 'g');
plot(links2(1,:),links2(2,:), 'Color', 'g');
plot(links3(1,:),links3(2,:), 'Color', 'g');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
axis equal
grid on
loop=loop+1;
end
figure(1);
figure(2);




%Calculating the workspace of the parallel robot for a give oriantatio
%alpha
  
Alpha=input(" \n Give orientation alpha for the workspace: ");
resolution=5;
limit=200;
CPP=zeros(2,3);
e1=zeros(1,3);
e2=zeros(1,3);
e3=zeros(1,3);
t=zeros(1,3);
theta=zeros(1,3);
PBPP=zeros(2,3);
n=1;
points=zeros(2,2*limit);
CentreX=-limit:resolution:limit;
CentreY=-limit:resolution:limit;

%calculating the theta values for each position of the central needle in
%the workspace
for j=1:length(CentreX)
    for k=1:length(CentreY)
        for i=1:3 
            CPP(1,i)=-rPlatform*cos((Alpha+30+120*(i-1))*(pi()/180))+CentreX(j);
            CPP(2,i)=-rPlatform*sin((Alpha+30+120*(i-1))*(pi()/180))+CentreY(k);
            PBPP(1,i)=PBB(1,i)+CPP(1,i);
            PBPP(2,i)=PBB(2,i)+CPP(2,i);
            e1(i)=-2*PBPP(2,i)*Sa;
            e2(i)=-2*PBPP(1,i)*Sa;
            e3(i)=(PBPP(1,i))^2+(PBPP(2,i))^2+Sa^2-L^2;
            t(i)=(-e1(i)-sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
            theta(i)=2*atan(t(i));
        end
        if isreal(theta)==1 
            points(1,n)=CentreX(j);
            points(2,n)=CentreY(k);
            n=n+1;
        end
    end
end

% plot the workspace of the parallel robot
figure(3);
base=[-PBB(1,:) -PBB(1,1);-PBB(2,:) -PBB(2,1)];

if points==0
else
    plot(points(1,:),points(2,:),'black.');
    hold on;
end

line(base(1,:),base(2,:), 'Color', 'blue');
plot(0,0,'blue+');
title("Workspace of Planar Parallel Robot with orientation: "+Alpha+"°"); xlabel('x (mm)') ; ylabel('y (mm)'); zlabel('z (mm)');
axis equal;
