function RobotGripperControl(serialMotorControl, motor, X)
%% ==========================Robot Head Control============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 3, 2014
%
% Controls the actual Mechatronic Arm using the DOF servos being controlled
% by an Arduino. Translates the Simulation calculated angles into Servo
% motor degree angles.

% Translate thetas into Mechatronic Arm controls
X(1) = (X(1));
X(2) = (X(2));

% Cut off the angles at 0 and 180 degrees
X(X > 180) = 180;
X(X < 0) = 0;

% Move the arm motors
serialMotorControl.servoWrite(motor.gripperLeft, round(X(1)));
serialMotorControl.servoWrite(motor.gripperRight, round(X(2)));