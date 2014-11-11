%% =====================RobotBodyVisual 3D Object==========================
% ME 5524: Bayesian Robotics
% Gerardo Bledt & James Burton
% Spring 2014 (5/1/2014)
%
% - Creates a RobotBodyVisual object that uses calculated vertex
%   coordinates to make a new visualization.

function RobotBodyVisual(V, lims, color)

surf1.x = [V(1,3), V(1,1), V(1,5), V(1,7)]'; % Top
surf1.y = [V(2,3), V(2,1), V(2,5), V(2,7)]';
surf1.z = [V(3,3), V(3,1), V(3,5), V(3,7)]';

surf2.x = [V(1,4), V(1,2), V(1,6), V(1,8)]'; % Bottom
surf2.y = [V(2,4), V(2,2), V(2,6), V(2,8)]';
surf2.z = [V(3,4), V(3,2), V(3,6), V(3,8)]';

surf3.x = [V(1,4), V(1,2), V(1,1), V(1,3)]'; % Right Back
surf3.y = [V(2,4), V(2,2), V(2,1), V(2,3)]';
surf3.z = [V(3,4), V(3,2), V(3,1), V(3,3)]';

surf4.x = [V(1,2), V(1,1), V(1,5), V(1,6)]'; % Left Back
surf4.y = [V(2,2), V(2,1), V(2,5), V(2,6)]';
surf4.z = [V(3,2), V(3,1), V(3,5), V(3,6)]';

surf5.x = [V(1,6), V(1,5), V(1,7), V(1,8)]'; % Left Front
surf5.y = [V(2,6), V(2,5), V(2,7), V(2,8)]';
surf5.z = [V(3,6), V(3,5), V(3,7), V(3,8)]';

surf6.x = [V(1,4), V(1,3), V(1,7), V(1,8)]'; % Right Front
surf6.y = [V(2,4), V(2,3), V(2,7), V(2,8)]';
surf6.z = [V(3,4), V(3,3), V(3,7), V(3,8)]';

Vertices.x = horzcat(surf1.x,surf2.x,surf3.x,surf4.x,surf5.x,surf6.x);
Vertices.y = horzcat(surf1.y,surf2.y,surf3.y,surf4.y,surf5.y,surf6.y);
Vertices.z = horzcat(surf1.z,surf2.z,surf3.z,surf4.z,surf5.z,surf6.z);

fill3(Vertices.x, Vertices.y, Vertices.z, color);
xlim(lims); ylim(lims); zlim(lims);
xlabel('x'); ylabel('y'); zlabel('z');
grid 