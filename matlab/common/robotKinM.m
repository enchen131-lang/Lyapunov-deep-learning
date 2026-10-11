function output = robotKinM(jointData, d8)

%% VREP parameters
d1 = 1.053; % 0.333 + 0.72
d3 = 0.316;
d5 = 0.384;
d7 = 0.107;

a4 = 0.0825;
a5 = -0.0825;
a7 = 0.088; % !!! Positive number !!!

d7 = d7 + d8;

q1 = jointData(1);
q2 = jointData(2);
q3 = jointData(3);
q4 = jointData(4);
q5 = jointData(5);
q6 = jointData(6);
q7 = jointData(7);

c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
c5 = cos(q5);
c6 = cos(q6);
c7 = cos(q7);

s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);
s5 = sin(q5);
s6 = sin(q6);
s7 = sin(q7);

%% Modified DH convention
A1 = [c1, -s1, 0, 0;
    s1, c1, 0, 0;
    0, 0, 1, d1;
    0, 0, 0, 1];

A2 = [c2, -s2, 0, 0;
    0, 0, 1, 0;
    -s2, -c2, 0, 0;
    0, 0, 0, 1];

A3 = [c3, -s3, 0, 0;
    0, 0, -1, -d3;
    s3, c3, 0, 0;
    0, 0, 0, 1];

A4 = [c4, -s4, 0, a4;
    0, 0, -1, 0;
    s4, c4, 0, 0;
    0, 0, 0, 1];

A5 = [c5, -s5, 0, a5;
    0, 0, 1, d5;
    -s5, -c5, 0, 0;
    0, 0, 0, 1];

A6 = [c6, -s6, 0, 0;
    0, 0, -1, 0;
    s6, c6, 0, 0;
    0, 0, 0, 1];

A7 = [c7, -s7, 0, a7;
    0, 0, -1, -d7;
    s7, c7, 0, 0;
    0, 0, 0, 1];

output = A1*A2*A3*A4*A5*A6*A7;

end
