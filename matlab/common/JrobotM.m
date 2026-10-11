function [J7, J8] = JrobotM(q, d8)
%JROBOTM Jacobians at the arm flange and the actual shaft-tip distance d8.

% d1 = 0.333; % With respect to the robot base
d1 = 1.053; % With respect to the world frame if the robot is attached to a table, 0.333 + 0.72
d3 = 0.316;
d5 = 0.384;
d7 = 0.107; % Without hand
% d7 = 0.2194; % With hand, 0.107 + 0.1034 + 0.009

a4 = 0.0825;
a5 = -0.0825;
a7 = 0.088; % !!! Positive number !!!

mys=sin(q);
s1=mys(1); s2=mys(2); s3=mys(3); s4=mys(4); s5=mys(5); s6=mys(6); s7=mys(7);
myc=cos(q);
c1=myc(1); c2=myc(2); c3=myc(3); c4=myc(4); c5=myc(5); c6=myc(6); c7=myc(7);

%% Modified DH convention
% A1 = [c1, -s1, 0, 0;
%     s1, c1, 0, 0;
%     0, 0, 1, d1;
%     0, 0, 0, 1];
% 
% A2 = [c2, -s2, 0, 0;
%     0, 0, 1, 0;
%     -s2, -c2, 0, 0;
%     0, 0, 0, 1];
% 
% A3 = [c3, -s3, 0, 0;
%     0, 0, -1, -d3;
%     s3, c3, 0, 0;
%     0, 0, 0, 1];
% 
% A4 = [c4, -s4, 0, a4;
%     0, 0, -1, 0;
%     s4, c4, 0, 0;
%     0, 0, 0, 1];
% 
% A5 = [c5, -s5, 0, a5;
%     0, 0, 1, d5;
%     -s5, -c5, 0, 0;
%     0, 0, 0, 1];
% 
% A6 = [c6, -s6, 0, 0;
%     0, 0, -1, 0;
%     s6, c6, 0, 0;
%     0, 0, 0, 1];
% 
% A7 = [c7, -s7, 0, a7;
%     0, 0, -1, -d7;
%     s7, c7, 0, 0;
%     0, 0, 0, 1];
% 
% A07 = A1*A2*A3*A4*A5*A6*A7;

%% J7 Jacobian of the shaft's start point
J7(1, 1) = d5*c1*s3*s4 - d3*s1*s2 - a4*c2*c3*s1 - a5*c1*c4*s3 - d5*c4*s1*s2 - a4*c1*s3 - a5*s1*s2*s4 + d5*c2*c3*s1*s4 + d7*c4*c6*s1*s2 - d7*c1*c6*s3*s4 - d7*c1*c3*s5*s6 - a7*c4*s1*s2*s6 + a7*c1*s3*s4*s6 - a5*c2*c3*c4*s1 - a7*c1*c3*c6*s5 - a7*c1*c4*c5*c6*s3 - d7*c2*c3*c6*s1*s4 - d7*c1*c4*c5*s3*s6 + a7*c2*c3*s1*s4*s6 + a7*c2*c6*s1*s3*s5 - a7*c5*c6*s1*s2*s4 + d7*c2*s1*s3*s5*s6 - d7*c5*s1*s2*s4*s6 - a7*c2*c3*c4*c5*c6*s1 - d7*c2*c3*c4*c5*s1*s6;
J7(2, 1) = d3*c1*s2 - a4*s1*s3 + a4*c1*c2*c3 + d5*c1*c4*s2 + a5*c1*s2*s4 - a5*c4*s1*s3 + d5*s1*s3*s4 - d5*c1*c2*c3*s4 - d7*c1*c4*c6*s2 + a7*c1*c4*s2*s6 - a7*c3*c6*s1*s5 - d7*c6*s1*s3*s4 - d7*c3*s1*s5*s6 + a7*s1*s3*s4*s6 + a5*c1*c2*c3*c4 + d7*c1*c2*c3*c6*s4 - a7*c1*c2*c3*s4*s6 - a7*c1*c2*c6*s3*s5 + a7*c1*c5*c6*s2*s4 - a7*c4*c5*c6*s1*s3 - d7*c1*c2*s3*s5*s6 + d7*c1*c5*s2*s4*s6 - d7*c4*c5*s1*s3*s6 + a7*c1*c2*c3*c4*c5*c6 + d7*c1*c2*c3*c4*c5*s6;
J7(3, 1) = 0.0;

J7(1, 2) = d3*c1*c2 + d5*c1*c2*c4 - a4*c1*c3*s2 + a5*c1*c2*s4 + d5*c1*c3*s2*s4 - d7*c1*c2*c4*c6 - a5*c1*c3*c4*s2 + a7*c1*c2*c4*s6 + a7*c1*c2*c5*c6*s4 - d7*c1*c3*c6*s2*s4 + d7*c1*c2*c5*s4*s6 + a7*c1*c3*s2*s4*s6 + a7*c1*c6*s2*s3*s5 + d7*c1*s2*s3*s5*s6 - a7*c1*c3*c4*c5*c6*s2 - d7*c1*c3*c4*c5*s2*s6;
J7(2, 2) = d3*c2*s1 + d5*c2*c4*s1 - a4*c3*s1*s2 + a5*c2*s1*s4 - d7*c2*c4*c6*s1 - a5*c3*c4*s1*s2 + a7*c2*c4*s1*s6 + d5*c3*s1*s2*s4 + a7*c2*c5*c6*s1*s4 - d7*c3*c6*s1*s2*s4 + d7*c2*c5*s1*s4*s6 + a7*c3*s1*s2*s4*s6 + a7*c6*s1*s2*s3*s5 + d7*s1*s2*s3*s5*s6 - a7*c3*c4*c5*c6*s1*s2 - d7*c3*c4*c5*s1*s2*s6;
J7(3, 2) = d5*c2*c3*s4 - a4*c2*c3 - d5*c4*s2 - a5*s2*s4 - a5*c2*c3*c4 - d3*s2 + d7*c4*c6*s2 - a7*c4*s2*s6 - d7*c2*c3*c6*s4 + a7*c2*c3*s4*s6 + a7*c2*c6*s3*s5 - a7*c5*c6*s2*s4 + d7*c2*s3*s5*s6 - d7*c5*s2*s4*s6 - a7*c2*c3*c4*c5*c6 - d7*c2*c3*c4*c5*s6;

J7(1, 3) = d5*c3*s1*s4 - a4*c1*c2*s3 - a5*c3*c4*s1 - a4*c3*s1 + d5*c1*c2*s3*s4 - d7*c3*c6*s1*s4 + a7*c3*s1*s4*s6 + a7*c6*s1*s3*s5 + d7*s1*s3*s5*s6 - a5*c1*c2*c4*s3 - a7*c1*c2*c3*c6*s5 - a7*c3*c4*c5*c6*s1 - d7*c1*c2*c6*s3*s4 - d7*c1*c2*c3*s5*s6 - d7*c3*c4*c5*s1*s6 + a7*c1*c2*s3*s4*s6 - a7*c1*c2*c4*c5*c6*s3 - d7*c1*c2*c4*c5*s3*s6;
J7(2, 3) = a4*c1*c3 + a5*c1*c3*c4 - d5*c1*c3*s4 - a4*c2*s1*s3 + d7*c1*c3*c6*s4 - a5*c2*c4*s1*s3 - a7*c1*c3*s4*s6 - a7*c1*c6*s3*s5 + d5*c2*s1*s3*s4 - d7*c1*s3*s5*s6 + a7*c1*c3*c4*c5*c6 + d7*c1*c3*c4*c5*s6 - a7*c2*c3*c6*s1*s5 - d7*c2*c6*s1*s3*s4 - d7*c2*c3*s1*s5*s6 + a7*c2*s1*s3*s4*s6 - a7*c2*c4*c5*c6*s1*s3 - d7*c2*c4*c5*s1*s3*s6;
J7(3, 3) = a4*s2*s3 + a5*c4*s2*s3 - d5*s2*s3*s4 + a7*c3*c6*s2*s5 + d7*c6*s2*s3*s4 + d7*c3*s2*s5*s6 - a7*s2*s3*s4*s6 + a7*c4*c5*c6*s2*s3 + d7*c4*c5*s2*s3*s6;

J7(1, 4) = a5*c1*c4*s2 - d5*c1*s2*s4 + d5*c4*s1*s3 + a5*s1*s3*s4 + d7*c1*c6*s2*s4 - d7*c4*c6*s1*s3 - a7*c1*s2*s4*s6 + a7*c4*s1*s3*s6 - d5*c1*c2*c3*c4 - a5*c1*c2*c3*s4 + d7*c1*c2*c3*c4*c6 - a7*c1*c2*c3*c4*s6 + a7*c1*c4*c5*c6*s2 + d7*c1*c4*c5*s2*s6 + a7*c5*c6*s1*s3*s4 + d7*c5*s1*s3*s4*s6 - a7*c1*c2*c3*c5*c6*s4 - d7*c1*c2*c3*c5*s4*s6;
J7(2, 4) = a5*c4*s1*s2 - d5*c1*c4*s3 - a5*c1*s3*s4 - d5*s1*s2*s4 - d5*c2*c3*c4*s1 + d7*c1*c4*c6*s3 - a5*c2*c3*s1*s4 - a7*c1*c4*s3*s6 + d7*c6*s1*s2*s4 - a7*s1*s2*s4*s6 + d7*c2*c3*c4*c6*s1 - a7*c2*c3*c4*s1*s6 + a7*c4*c5*c6*s1*s2 - a7*c1*c5*c6*s3*s4 + d7*c4*c5*s1*s2*s6 - d7*c1*c5*s3*s4*s6 - a7*c2*c3*c5*c6*s1*s4 - d7*c2*c3*c5*s1*s4*s6;
J7(3, 4) = a5*c2*c4 - d5*c2*s4 + d5*c3*c4*s2 + d7*c2*c6*s4 + a5*c3*s2*s4 - a7*c2*s4*s6 - d7*c3*c4*c6*s2 + d7*c2*c4*c5*s6 + a7*c3*c4*s2*s6 + a7*c2*c4*c5*c6 + a7*c3*c5*c6*s2*s4 + d7*c3*c5*s2*s4*s6;

J7(1, 5) = a7*c4*c6*s1*s3*s5 - a7*c3*c5*c6*s1 - a7*c1*c2*c5*c6*s3 - d7*c1*c2*c5*s3*s6 - a7*c1*c6*s2*s4*s5 - d7*c3*c5*s1*s6 - d7*c1*s2*s4*s5*s6 + d7*c4*s1*s3*s5*s6 - a7*c1*c2*c3*c4*c6*s5 - d7*c1*c2*c3*c4*s5*s6;
J7(2, 5) = d7*c1*c3*c5*s6 + a7*c1*c3*c5*c6 - a7*c2*c5*c6*s1*s3 - a7*c1*c4*c6*s3*s5 - d7*c2*c5*s1*s3*s6 - d7*c1*c4*s3*s5*s6 - a7*c6*s1*s2*s4*s5 - d7*s1*s2*s4*s5*s6 - a7*c2*c3*c4*c6*s1*s5 - d7*c2*c3*c4*s1*s5*s6;
J7(3, 5) = a7*c5*c6*s2*s3 - a7*c2*c6*s4*s5 + d7*c5*s2*s3*s6 - d7*c2*s4*s5*s6 + a7*c3*c4*c6*s2*s5 + d7*c3*c4*s2*s5*s6;

J7(1, 6) = d7*c1*c4*s2*s6 - d7*c3*c6*s1*s5 + a7*c6*s1*s3*s4 + a7*c3*s1*s5*s6 + d7*s1*s3*s4*s6 + a7*c1*c4*c6*s2 - a7*c1*c2*c3*c6*s4 - d7*c1*c2*c3*s4*s6 - d7*c1*c2*c6*s3*s5 + d7*c1*c5*c6*s2*s4 - d7*c4*c5*c6*s1*s3 + a7*c1*c2*s3*s5*s6 - a7*c1*c5*s2*s4*s6 + a7*c4*c5*s1*s3*s6 + d7*c1*c2*c3*c4*c5*c6 - a7*c1*c2*c3*c4*c5*s6;
J7(2, 6) = d7*c1*c3*c6*s5 + a7*c4*c6*s1*s2 - a7*c1*c6*s3*s4 - a7*c1*c3*s5*s6 + d7*c4*s1*s2*s6 - d7*c1*s3*s4*s6 + d7*c1*c4*c5*c6*s3 - a7*c2*c3*c6*s1*s4 - a7*c1*c4*c5*s3*s6 - d7*c2*c3*s1*s4*s6 - d7*c2*c6*s1*s3*s5 + d7*c5*c6*s1*s2*s4 + a7*c2*s1*s3*s5*s6 - a7*c5*s1*s2*s4*s6 + d7*c2*c3*c4*c5*c6*s1 - a7*c2*c3*c4*c5*s1*s6;
J7(3, 6) = a7*c2*c4*c6 + d7*c2*c4*s6 + d7*c2*c5*c6*s4 + a7*c3*c6*s2*s4 - a7*c2*c5*s4*s6 + d7*c3*s2*s4*s6 + d7*c6*s2*s3*s5 - a7*s2*s3*s5*s6 - d7*c3*c4*c5*c6*s2 + a7*c3*c4*c5*s2*s6;

J7(1, 7) = 0.0;
J7(2, 7) = 0.0;
J7(3, 7) = 0.0;

%% J8 Jacobian of the shaft's end point (end tip of disk 1)
% A8 = [1 0 0 0;...
%     0 1 0 0;...
%     0 0 1 d8;...
%     0 0 0 1];
% A08 = A07*A8;

d7 = d7 + d8; % 0.107 + 0.3

J8(1, 1) = d5*c1*s3*s4 - d3*s1*s2 - a4*c2*c3*s1 - a5*c1*c4*s3 - d5*c4*s1*s2 - a4*c1*s3 - a5*s1*s2*s4 + d5*c2*c3*s1*s4 + d7*c4*c6*s1*s2 - d7*c1*c6*s3*s4 - d7*c1*c3*s5*s6 - a7*c4*s1*s2*s6 + a7*c1*s3*s4*s6 - a5*c2*c3*c4*s1 - a7*c1*c3*c6*s5 - a7*c1*c4*c5*c6*s3 - d7*c2*c3*c6*s1*s4 - d7*c1*c4*c5*s3*s6 + a7*c2*c3*s1*s4*s6 + a7*c2*c6*s1*s3*s5 - a7*c5*c6*s1*s2*s4 + d7*c2*s1*s3*s5*s6 - d7*c5*s1*s2*s4*s6 - a7*c2*c3*c4*c5*c6*s1 - d7*c2*c3*c4*c5*s1*s6;
J8(2, 1) = d3*c1*s2 - a4*s1*s3 + a4*c1*c2*c3 + d5*c1*c4*s2 + a5*c1*s2*s4 - a5*c4*s1*s3 + d5*s1*s3*s4 - d5*c1*c2*c3*s4 - d7*c1*c4*c6*s2 + a7*c1*c4*s2*s6 - a7*c3*c6*s1*s5 - d7*c6*s1*s3*s4 - d7*c3*s1*s5*s6 + a7*s1*s3*s4*s6 + a5*c1*c2*c3*c4 + d7*c1*c2*c3*c6*s4 - a7*c1*c2*c3*s4*s6 - a7*c1*c2*c6*s3*s5 + a7*c1*c5*c6*s2*s4 - a7*c4*c5*c6*s1*s3 - d7*c1*c2*s3*s5*s6 + d7*c1*c5*s2*s4*s6 - d7*c4*c5*s1*s3*s6 + a7*c1*c2*c3*c4*c5*c6 + d7*c1*c2*c3*c4*c5*s6;
J8(3, 1) = 0.0;

J8(1, 2) = d3*c1*c2 + d5*c1*c2*c4 - a4*c1*c3*s2 + a5*c1*c2*s4 + d5*c1*c3*s2*s4 - d7*c1*c2*c4*c6 - a5*c1*c3*c4*s2 + a7*c1*c2*c4*s6 + a7*c1*c2*c5*c6*s4 - d7*c1*c3*c6*s2*s4 + d7*c1*c2*c5*s4*s6 + a7*c1*c3*s2*s4*s6 + a7*c1*c6*s2*s3*s5 + d7*c1*s2*s3*s5*s6 - a7*c1*c3*c4*c5*c6*s2 - d7*c1*c3*c4*c5*s2*s6;
J8(2, 2) = d3*c2*s1 + d5*c2*c4*s1 - a4*c3*s1*s2 + a5*c2*s1*s4 - d7*c2*c4*c6*s1 - a5*c3*c4*s1*s2 + a7*c2*c4*s1*s6 + d5*c3*s1*s2*s4 + a7*c2*c5*c6*s1*s4 - d7*c3*c6*s1*s2*s4 + d7*c2*c5*s1*s4*s6 + a7*c3*s1*s2*s4*s6 + a7*c6*s1*s2*s3*s5 + d7*s1*s2*s3*s5*s6 - a7*c3*c4*c5*c6*s1*s2 - d7*c3*c4*c5*s1*s2*s6;
J8(3, 2) = d5*c2*c3*s4 - a4*c2*c3 - d5*c4*s2 - a5*s2*s4 - a5*c2*c3*c4 - d3*s2 + d7*c4*c6*s2 - a7*c4*s2*s6 - d7*c2*c3*c6*s4 + a7*c2*c3*s4*s6 + a7*c2*c6*s3*s5 - a7*c5*c6*s2*s4 + d7*c2*s3*s5*s6 - d7*c5*s2*s4*s6 - a7*c2*c3*c4*c5*c6 - d7*c2*c3*c4*c5*s6;

J8(1, 3) = d5*c3*s1*s4 - a4*c1*c2*s3 - a5*c3*c4*s1 - a4*c3*s1 + d5*c1*c2*s3*s4 - d7*c3*c6*s1*s4 + a7*c3*s1*s4*s6 + a7*c6*s1*s3*s5 + d7*s1*s3*s5*s6 - a5*c1*c2*c4*s3 - a7*c1*c2*c3*c6*s5 - a7*c3*c4*c5*c6*s1 - d7*c1*c2*c6*s3*s4 - d7*c1*c2*c3*s5*s6 - d7*c3*c4*c5*s1*s6 + a7*c1*c2*s3*s4*s6 - a7*c1*c2*c4*c5*c6*s3 - d7*c1*c2*c4*c5*s3*s6;
J8(2, 3) = a4*c1*c3 + a5*c1*c3*c4 - d5*c1*c3*s4 - a4*c2*s1*s3 + d7*c1*c3*c6*s4 - a5*c2*c4*s1*s3 - a7*c1*c3*s4*s6 - a7*c1*c6*s3*s5 + d5*c2*s1*s3*s4 - d7*c1*s3*s5*s6 + a7*c1*c3*c4*c5*c6 + d7*c1*c3*c4*c5*s6 - a7*c2*c3*c6*s1*s5 - d7*c2*c6*s1*s3*s4 - d7*c2*c3*s1*s5*s6 + a7*c2*s1*s3*s4*s6 - a7*c2*c4*c5*c6*s1*s3 - d7*c2*c4*c5*s1*s3*s6;
J8(3, 3) = a4*s2*s3 + a5*c4*s2*s3 - d5*s2*s3*s4 + a7*c3*c6*s2*s5 + d7*c6*s2*s3*s4 + d7*c3*s2*s5*s6 - a7*s2*s3*s4*s6 + a7*c4*c5*c6*s2*s3 + d7*c4*c5*s2*s3*s6;

J8(1, 4) = a5*c1*c4*s2 - d5*c1*s2*s4 + d5*c4*s1*s3 + a5*s1*s3*s4 + d7*c1*c6*s2*s4 - d7*c4*c6*s1*s3 - a7*c1*s2*s4*s6 + a7*c4*s1*s3*s6 - d5*c1*c2*c3*c4 - a5*c1*c2*c3*s4 + d7*c1*c2*c3*c4*c6 - a7*c1*c2*c3*c4*s6 + a7*c1*c4*c5*c6*s2 + d7*c1*c4*c5*s2*s6 + a7*c5*c6*s1*s3*s4 + d7*c5*s1*s3*s4*s6 - a7*c1*c2*c3*c5*c6*s4 - d7*c1*c2*c3*c5*s4*s6;
J8(2, 4) = a5*c4*s1*s2 - d5*c1*c4*s3 - a5*c1*s3*s4 - d5*s1*s2*s4 - d5*c2*c3*c4*s1 + d7*c1*c4*c6*s3 - a5*c2*c3*s1*s4 - a7*c1*c4*s3*s6 + d7*c6*s1*s2*s4 - a7*s1*s2*s4*s6 + d7*c2*c3*c4*c6*s1 - a7*c2*c3*c4*s1*s6 + a7*c4*c5*c6*s1*s2 - a7*c1*c5*c6*s3*s4 + d7*c4*c5*s1*s2*s6 - d7*c1*c5*s3*s4*s6 - a7*c2*c3*c5*c6*s1*s4 - d7*c2*c3*c5*s1*s4*s6;
J8(3, 4) = a5*c2*c4 - d5*c2*s4 + d5*c3*c4*s2 + d7*c2*c6*s4 + a5*c3*s2*s4 - a7*c2*s4*s6 - d7*c3*c4*c6*s2 + d7*c2*c4*c5*s6 + a7*c3*c4*s2*s6 + a7*c2*c4*c5*c6 + a7*c3*c5*c6*s2*s4 + d7*c3*c5*s2*s4*s6;

J8(1, 5) = a7*c4*c6*s1*s3*s5 - a7*c3*c5*c6*s1 - a7*c1*c2*c5*c6*s3 - d7*c1*c2*c5*s3*s6 - a7*c1*c6*s2*s4*s5 - d7*c3*c5*s1*s6 - d7*c1*s2*s4*s5*s6 + d7*c4*s1*s3*s5*s6 - a7*c1*c2*c3*c4*c6*s5 - d7*c1*c2*c3*c4*s5*s6;
J8(2, 5) = d7*c1*c3*c5*s6 + a7*c1*c3*c5*c6 - a7*c2*c5*c6*s1*s3 - a7*c1*c4*c6*s3*s5 - d7*c2*c5*s1*s3*s6 - d7*c1*c4*s3*s5*s6 - a7*c6*s1*s2*s4*s5 - d7*s1*s2*s4*s5*s6 - a7*c2*c3*c4*c6*s1*s5 - d7*c2*c3*c4*s1*s5*s6;
J8(3, 5) = a7*c5*c6*s2*s3 - a7*c2*c6*s4*s5 + d7*c5*s2*s3*s6 - d7*c2*s4*s5*s6 + a7*c3*c4*c6*s2*s5 + d7*c3*c4*s2*s5*s6;

J8(1, 6) = d7*c1*c4*s2*s6 - d7*c3*c6*s1*s5 + a7*c6*s1*s3*s4 + a7*c3*s1*s5*s6 + d7*s1*s3*s4*s6 + a7*c1*c4*c6*s2 - a7*c1*c2*c3*c6*s4 - d7*c1*c2*c3*s4*s6 - d7*c1*c2*c6*s3*s5 + d7*c1*c5*c6*s2*s4 - d7*c4*c5*c6*s1*s3 + a7*c1*c2*s3*s5*s6 - a7*c1*c5*s2*s4*s6 + a7*c4*c5*s1*s3*s6 + d7*c1*c2*c3*c4*c5*c6 - a7*c1*c2*c3*c4*c5*s6;
J8(2, 6) = d7*c1*c3*c6*s5 + a7*c4*c6*s1*s2 - a7*c1*c6*s3*s4 - a7*c1*c3*s5*s6 + d7*c4*s1*s2*s6 - d7*c1*s3*s4*s6 + d7*c1*c4*c5*c6*s3 - a7*c2*c3*c6*s1*s4 - a7*c1*c4*c5*s3*s6 - d7*c2*c3*s1*s4*s6 - d7*c2*c6*s1*s3*s5 + d7*c5*c6*s1*s2*s4 + a7*c2*s1*s3*s5*s6 - a7*c5*s1*s2*s4*s6 + d7*c2*c3*c4*c5*c6*s1 - a7*c2*c3*c4*c5*s1*s6;
J8(3, 6) = a7*c2*c4*c6 + d7*c2*c4*s6 + d7*c2*c5*c6*s4 + a7*c3*c6*s2*s4 - a7*c2*c5*s4*s6 + d7*c3*s2*s4*s6 + d7*c6*s2*s3*s5 - a7*s2*s3*s5*s6 - d7*c3*c4*c5*c6*s2 + a7*c3*c4*c5*s2*s6;

J8(1, 7) = 0.0;
J8(2, 7) = 0.0;
J8(3, 7) = 0.0;

J8(4, 1) = 0.0;
J8(5, 1) = 0.0;
J8(6, 1) = 1.0;

J8(4, 2) = -s1;
J8(5, 2) = c1;
J8(6, 2) = 0.0;
 
J8(4, 3) = c1*s2;
J8(5, 3) = s1*s2;
J8(6, 3) = c2;
 
J8(4, 4) = c3*s1 + c1*c2*s3;
J8(5, 4) = c2*s1*s3 - c1*c3;
J8(6, 4) = -s2*s3;

J8(4, 5) = s4*(s1*s3 - c1*c2*c3) + c1*c4*s2;
J8(5, 5) = c4*s1*s2 - s4*(c1*s3 + c2*c3*s1);
J8(6, 5) = c2*c4 + c3*s2*s4;

J8(4, 6) = c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4);
J8(5, 6) = s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3);
J8(6, 6) = s5*(c2*s4 - c3*c4*s2) - c5*s2*s3; 

J8(4, 7) = - s6*(s5*(c3*s1 + c1*c2*s3) + c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)) - c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2);
J8(5, 7) = s6*(s5*(c1*c3 - c2*s1*s3) + c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)) + c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
J8(6, 7) = s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - c6*(c2*c4 + c3*s2*s4);

end
