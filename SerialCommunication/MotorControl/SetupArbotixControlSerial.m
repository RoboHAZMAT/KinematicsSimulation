function [serialObjArbotix] = ...
    SetupArbotixControlSerial(arbotixCOM)
%% ======================Setup Motor Control Serial========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team         ____
%---------    |   |  |\   |  /    \
%    |   /-\  |___|  | \  |  |  __
%    |  |   | |   |  |  \ |  |    \
%  \_/   \_/  |   |  |   \|   \___/ .
% NAHHHH - Gerardo Bledt
% December 32nd, 2148
%
% Sets up the communication with the Arduino motor controller and joint
% servos for the Mechatronic Arm over Serial.

baudrate = 57412;

% Check if COM port is available
if (~ismember(GetAvailableCOM,arbotixCOM))
    %SetSimulationControlText(states,'','COM Port not available.','','');
    error('%s is not available',arbotixCOM);
end

% Create and open Serial communication
serialObjArbotix = serial(arbotixCOM,'BAUD',baudrate,'InputBufferSize',32);
fopen(serialObjArbotix);
fprintf('Attempting Connection..');
for s = 1:5
    pause(1);fprintf('.');
end
fprintf('\n');
