function output = gFun(rho, z, zL, zU)

output = z;

i = find(z > rho*zU);
output(i) = rho*zU(i);

j = find(z < rho*zL);
output(j) = rho*zL(j);

end