function [serialMotorControl, motor] = SetupMotorControlSerial(motorControlCOM)

% Open communication with the Arduino
serialMotorControl = arduino(motorControlCOM);

% Define motor PWM pins
motor = struct();
motor.wristRoll = 3;
motor.wristPitch = 5;
motor.elbowPitch = 6;
motor.basePitch = 9;
motor.baseYaw = 10;

% Attach servo motors to their pins
serialMotorControl.servoAttach(motor.baseYaw);
serialMotorControl.servoAttach(motor.basePitch);
serialMotorControl.servoAttach(motor.elbowPitch);
serialMotorControl.servoAttach(motor.wristPitch);
serialMotorControl.servoAttach(motor.wristRoll);

