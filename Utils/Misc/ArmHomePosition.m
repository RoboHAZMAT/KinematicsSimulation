function homePosition = ArmHomePosition(arm)
homePosition = '';
if (strcmpi(arm,'r'))
    homePosition = [pi/8;pi/8;-pi/6;-pi/2;0;0];
elseif (strcmpi(arm,'l'))
    homePosition = [pi/8;-pi/8;pi/6;-pi/2;0;0];
end