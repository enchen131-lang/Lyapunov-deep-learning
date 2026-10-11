function output = Jimage(u, v, u0, v0, f, rho, Z)

u_bar = u - u0;
v_bar = v - v0;

f_bar = f/rho;

output = [-f_bar/Z, 0, u_bar/Z, u_bar*v_bar/f_bar, -(f_bar^2 + u_bar^2)/f_bar, v_bar;...
         0, -f_bar/Z, v_bar/Z, (f_bar^2 + v_bar^2)/f_bar, -u_bar*v_bar/f_bar, -u_bar];

end