function [nIMUR, nIMUL, nIMUH] = SetWirelessIMU(rightArm, leftArm, head)
nIMUR = [];
nIMUL = [];
nIMUH = [];

if (rightArm); nIMUR = ['2','3']; end;
if (leftArm);  nIMUL = ['4','5']; end;
if (head); nIMUH = '6'; end;