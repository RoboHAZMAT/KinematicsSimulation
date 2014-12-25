function psi = yawOffset(vector)

mag = sqrt(vector(1,1)^2 + vector(1,2)^2);

if (vector(1,2) >= 0)
    psi = acos(vector(1,1)/mag);
elseif (vector(1,2) < 0)
    psi = -acos(vector(1,1)/mag);
end