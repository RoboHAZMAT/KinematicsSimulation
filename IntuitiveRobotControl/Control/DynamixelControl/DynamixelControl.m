function DynamixelControl(dynamixel, X, arm)
if (strcmpi(arm,'r'))
    X(1) = -X(1) + 125*pi/180;
    X(2) = -X(2) + (125+90)*pi/180;
    X(3) = X(3) + 125*pi/180;
elseif (strcmpi(arm,'l'))
        
end
% Same code to control both arms
%for i = 1:length(dynamixel) 
for i = 1:3
    dynamixel{i}.setPositionEX(X(i),'rad');
end