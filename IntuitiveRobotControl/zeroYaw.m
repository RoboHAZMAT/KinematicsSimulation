function [vector, psi] = ZeroYaw(vector, psi, reset)

% Calculate the yaw offset if requested
if (reset)
    % Magnitude of the vector
    mag = sqrt(vector(1,1)^2 + vector(1,2)^2);
    
    % The yaw offset of the vector from 0
    if (vector(1,2) >= 0)
        psi = acos(vector(1,1)/mag);
    elseif (vector(1,2) < 0)
        psi = -acos(vector(1,1)/mag);
    end
end

% Calculates the rotation matrix to zero the yaw
R = [cos(psi), sin(psi), 0;
    -sin(psi), cos(psi), 0;
    0,        0, 1];

% Readjusts the vector
vector = (R*vector')';