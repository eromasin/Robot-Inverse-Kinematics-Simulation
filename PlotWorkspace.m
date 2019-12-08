%distal DH
a1 = 0;     alpha1 = 0;     d1 = 3; %t1 = 0   %shoulder1 to base
a2 = 0;     alpha2 = pi/2;  d2 = 0; %t2 = 0   %shoulder2 to shoulder1
a3 = 5.75;  alpha3 = 0;     d3 = 0; %t3 = 0   %elbow to shoulder2
a4 = 7.375; alpha4 = 0;     d4 = 0; %t4 = 0   %wrist1 to elbow
a5 = 0;     alpha5 = -pi/2; d5 = 2; %t5 = 0   %wrist2 to wrist1

%set joints limits
t1_min = -pi/2; t1_max = pi/2;
t2_min = -pi/2; t2_max = pi/2;
t3_min = -pi/2; t3_max = pi/2;
t4_min = -pi/2; t4_max = pi/2;
t5_min = -pi/2; t5_max = pi/2;

N = 5000; 
% %pick random value between max and min
% t1 = t1_min + (t1_max-t1_min)*rand(N,1); 
% t2 = t2_min + (t2_max-t2_min)*rand(N,1); 
% t3 = t3_min + (t3_max-t3_min)*rand(N,1); 
% t4 = t4_min + (t4_max-t4_min)*rand(N,1); 
% t5 = t5_min + (t5_max-t5_min)*rand(N,1); 

%iterate through values of end effector to make cloud
for i = 1:N  
    A1 = TransMat(a1,alpha1,d1,t1(i));  
    A2 = TransMat(a2,alpha2,d2,t2(i));  
    A3 = TransMat(a3,alpha3,d3,t3(i));  
    A4 = TransMat(a4,alpha4,d4,t4(i));  
    A5 = TransMat(a5,alpha5,d5,t5(i));  
    T = A1*A2*A3*A4*A5;  
    X=T(1,4);  
    Y=T(2,4);  
    Z=T(3,4);
    plot3(X,Y,Z,'.')  
    hold on; 
end 

% show views
% view(3); 
% title('Isometric view'); 
% xlabel('x (m)'); 
% ylabel('y (m)'); 
% zlabel('z (m) ');   

% view(2); % top view 
% title(' Top view'); 
% xlabel('x (m)'); 
% ylabel('y (m)'); 
% 
% view([1 0 0]); % y-z plane 
% title('Side view, Y-Z'); 
% ylabel('y (m)'); 
% zlabel('z (m)'); 
 
