function vector = zeroYaw(vector, psi)

R = [cos(psi), sin(psi), 0;
    -sin(psi), cos(psi), 0;
            0,        0, 1];

vector = (R*vector')';