%% ========================RobotBody 3D Object=============================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (5/1/2014)
%
% - Creates a RobotBody object that is a 3D rectangular prism object that 
%   has a definable (x, y, z) center for the object, definable x, y, and z
%   side lengths, and an RGB color vector. 
% 
% - Returns a 3x8 vector of vertex locations in (x,y,z) coordinates, the
%   calculated limits, and the color.

function [V, lims, color] = RobotBody
xc = -0.475; yc = 0; zc = 0; 
xlen = 1; ylen = 0.75; zlen = 0.1;
color = [.7,.7,.7];

surf1.x = [xc+xlen/2, xc+xlen/2, xc-xlen/2, xc-xlen/2]'; % Top
surf1.y = [yc-ylen/2, yc+ylen/2, yc+ylen/2, yc-ylen/2]';
surf1.z = [zc+zlen/2, zc+zlen/2, zc+zlen/2, zc+zlen/2]';

surf2.x = [xc+xlen/2, xc+xlen/2, xc-xlen/2, xc-xlen/2]'; % Bottom
surf2.y = [yc-ylen/2, yc+ylen/2, yc+ylen/2, yc-ylen/2]';
surf2.z = [zc-zlen/2, zc-zlen/2, zc-zlen/2, zc-zlen/2]';

surf3.x = [xc+xlen/2, xc+xlen/2, xc+xlen/2, xc+xlen/2]'; % Right Back
surf3.y = [yc-ylen/2, yc+ylen/2, yc+ylen/2, yc-ylen/2]';
surf3.z = [zc-zlen/2, zc-zlen/2, zc+zlen/2, zc+zlen/2]';

surf4.x = [xc+xlen/2, xc+xlen/2, xc-xlen/2, xc-xlen/2]'; % Left Back
surf4.y = [yc+ylen/2, yc+ylen/2, yc+ylen/2, yc+ylen/2]';
surf4.z = [zc-zlen/2, zc+zlen/2, zc+zlen/2, zc-zlen/2]';

surf5.x = [xc-xlen/2, xc-xlen/2, xc-xlen/2, xc-xlen/2]'; % Left Front
surf5.y = [yc+ylen/2, yc+ylen/2, yc-ylen/2, yc-ylen/2]';
surf5.z = [zc-zlen/2, zc+zlen/2, zc+zlen/2, zc-zlen/2]';

surf6.x = [xc+xlen/2, xc+xlen/2, xc-xlen/2, xc-xlen/2]'; % Right Front
surf6.y = [yc-ylen/2, yc-ylen/2, yc-ylen/2, yc-ylen/2]';
surf6.z = [zc-zlen/2, zc+zlen/2, zc+zlen/2, zc-zlen/2]';

Vertices.x = horzcat(surf1.x,surf2.x,surf3.x,surf4.x,surf5.x,surf6.x);
Vertices.y = horzcat(surf1.y,surf2.y,surf3.y,surf4.y,surf5.y,surf6.y);
Vertices.z = horzcat(surf1.z,surf2.z,surf3.z,surf4.z,surf5.z,surf6.z);

fill3(Vertices.x, Vertices.y, Vertices.z, color);
sf = max([xlen/2,ylen/2,zlen/2])*1.5; lims = [-sf, sf];
xlim(lims); ylim(lims); zlim(lims);
xlabel('x'); ylabel('y'); zlabel('z');  
V = [xc+xlen/2, xc+xlen/2, xc+xlen/2, xc+xlen/2, ...
    xc-xlen/2, xc-xlen/2, xc-xlen/2, xc-xlen/2; ...
    yc+ylen/2, yc+ylen/2, yc-ylen/2, yc-ylen/2, ...
    yc+ylen/2, yc+ylen/2, yc-ylen/2, yc-ylen/2; ...
    zc+zlen/2, zc-zlen/2, zc+zlen/2, zc-zlen/2, ...
    zc+zlen/2, zc-zlen/2, zc+zlen/2, zc-zlen/2];
grid on




