function [dynamixelRightArm, dynamixelLeftArm] = DynamixelControlSetup
% Creates and sets up all 12 Dynamixel servo motors and stores them as
% separate right and left are cells.
loadlibrary('dynamixel','dynamixel.h');
args = struct();
fprintf('\nDynamixel Connection Setup...\n');

%% ==============================Right Arm=================================
% ---------------Shoulder---------------
args.name = 'Right Shoulder Pitch';
args.id = 1;
args.portNum = 28;
args.maxPosition = 4095;
args.minPosition = 1500;
dynamixelRightArm{1} = Dynamixel(args);

args.name = 'Right Shoulder Roll';
args.id = 2;
args.maxPosition = 3675;
args.minPosition = 900;
dynamixelRightArm{2} = Dynamixel(args);

args.name = 'Right Shoulder Yaw';
args.id = 3;
args.maxPosition = 4095;
args.minPosition = 0;
dynamixelRightArm{3} = Dynamixel(args);

for i = 1:length(dynamixelRightArm)
    dynamixelRightArm{i}.setComplianceSlopes();
end

%% ==============================Left Arm==================================

% ---------------Shoulder---------------
args.name = 'Left Shoulder Pitch';
args.id = 11;
args.maxPosition = 4095;
args.minPosition = 0;
dynamixelLeftArm{1} = Dynamixel(args);

args.name = 'Left Shoulder Roll';
args.id = 12;
args.maxPosition = 4095;
args.minPosition = 0;
dynamixelLeftArm{2} = Dynamixel(args);

args.name = 'Left Shoulder Yaw';
args.id = 13;
args.maxPosition = 4095;
args.minPosition = 0;
dynamixelLeftArm{3} = Dynamixel(args);

for i = 1:length(dynamixelLeftArm)
    dynamixelLeftArm{i}.setComplianceSlopes();
end

fprintf('\nDynamixel Setup Complete.\n\n');