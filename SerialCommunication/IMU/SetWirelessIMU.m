function [nIMUR, nIMUL, nIMUH, receiveIMU] = SetWirelessIMU(rightArm, leftArm, head)
nIMUR = [];
nIMUL = [];
nIMUH = [];
receiveIMU = '00000';

if (rightArm); nIMUR = ['2','3']; receiveIMU(1) = '1'; receiveIMU(2) = '1'; end;
if (leftArm);  nIMUL = ['4','5']; receiveIMU(3) = '1'; receiveIMU(4) = '1'; end;
if (head); nIMUH = '6'; receiveIMU(5) = '1'; end;