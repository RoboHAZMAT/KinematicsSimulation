function MechatronicArmControl(serialMotorControl, motor, X)
%% =======================Mechatronic Arm Control==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 3, 2014
%
% Controls the actual Mechatronic Arm using the DOF servos being controlled
% by an Arduino. Translates the Simulation calculated angles into Servo
% motor degree angles.

% Translate thetas into Mechatronic Arm controls
X = X*180/pi;
X(1) = (-X(1) + 100);
X(2) = (-X(2) + 88);
X(3) = (X(3) + 80);
X(4) = (X(4) - 8);
X(5) = (X(5) + 108);

% Cut off the angles at 0 and 180 degrees
X(X > 180) = 180;
X(X < 0) = 0;

% Move the arm motors
serialMotorControl.servoWrite(motor.baseYaw, round(X(1)));
serialMotorControl.servoWrite(motor.basePitch, round(X(2)));
serialMotorControl.servoWrite(motor.elbowPitch, round(X(3)));
serialMotorControl.servoWrite(motor.wristPitch, round(X(4)));
serialMotorControl.servoWrite(motor.wristRoll, round(X(5)));