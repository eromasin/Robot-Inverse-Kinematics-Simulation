function [ T ] = TranMat( a,b,c,d )  
T = [ cos(d)  -sin(d)*cos(b)  sin(d)*sin(b)  a*cos(d); 
      sin(d)   cos(d)*cos(b) -cos(d)*sin(b)  a*sin(d); 
      0        sin(b)         cos(b)         c; 
      0        0              0              1]; 
end 
