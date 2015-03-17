function DynamixelControl(dynamixel, X, arm)
if (strcmpi(arm,'r'))
    X(1) = -X(1) + pi;
    X(2) = -X(2) + pi;
    X(4) = -X(4)+pi;
elseif (strcmpi(arm,'l'))
        
end
% Same code to control both arms
%for i = 1:length(dynamixel) 
for i = 4:5
    dynamixel{i}.setPosition(X(i),'rad');
end