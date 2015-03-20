function [dynamixelRightArm, dynamixelLeftArm] = DynamixelControlSetup
% Creates and sets up all 12 Dynamixel servo motors and stores them as
% separate right and left are cells.
loadlibrary('dynamixel','dynamixel.h');
args = struct();

%% ==============================Right Arm=================================
% ---------------Shoulder---------------
args.name = 'Right Shoulder Pitch';
args.id = 1;
args.portNum = 19;
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
%{
% ---------------Elbow---------------
args.name = 'Right Elbow Pitch';
args.id = 4;
args.portNum = 18;
args.maxPosition = 3325;
args.minPosition = 2047;
dynamixelRightArm{4} = Dynamixel(args);

args.name = 'Right Elbow Yaw';
args.id = 5;
args.maxPosition = 2047;
args.minPosition = 0;
dynamixelRightArm{5} = Dynamixel(args);

% ---------------Wrist---------------
args.name = 'Right Wrist Pitch';
args.id = 6;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelRightArm{6} = Dynamixel(args);

%% ==============================Left Arm==================================

% ---------------Shoulder---------------
args.name = 'Left Shoulder Pitch';
args.id = 11;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{1} = Dynamixel(args);

args.name = 'Left Shoulder Roll';
args.id = 12;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{2} = Dynamixel(args);

args.name = 'Left Shoulder Yaw';
args.id = 13;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{3} = Dynamixel(args);

% ---------------Elbow---------------
args.name = 'Left Elbow Pitch';
args.id = 14;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{4} = Dynamixel(args);

args.name = 'Left Elbow Yaw';
args.id = 15;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{5} = Dynamixel(args);

% ---------------Wrist---------------
args.name = 'Left Wrist Pitch';
args.id = 16;
args.maxPosition = 3047;
args.minPosition = 1047;
dynamixelLeftArm{6} = Dynamixel(args);
%}