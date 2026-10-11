function output = softH(jointAngle, len)
% phi is the pan joint angle
% theta is the tilt joint angle
% len is the length of the soft joint

phi = jointAngle(1);
theta = jointAngle(2);

if theta == 0
    r = len;
    output = [sin(phi)^2 + cos(phi)^2*cos(theta), cos(phi)*cos(theta)*sin(phi) - cos(phi)*sin(phi), cos(phi)*sin(theta), 0;...
        cos(phi)*cos(theta)*sin(phi) - cos(phi)*sin(phi), cos(phi)^2 + cos(theta)*sin(phi)^2, sin(phi)*sin(theta), 0;...
        -cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), r;...
        0, 0, 0, 1];
else
    r = len/theta;
    output = [sin(phi)^2 + cos(phi)^2*cos(theta), cos(phi)*cos(theta)*sin(phi) - cos(phi)*sin(phi), cos(phi)*sin(theta), -r*cos(phi)*(cos(theta) - 1);...
        cos(phi)*cos(theta)*sin(phi) - cos(phi)*sin(phi), cos(phi)^2 + cos(theta)*sin(phi)^2, sin(phi)*sin(theta), -r*sin(phi)*(cos(theta) - 1);...
        -cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), r*sin(theta);...
        0, 0, 0, 1];
end

end