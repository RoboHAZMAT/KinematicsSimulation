function calibrated = CalibrateIMU(serialObjIMU)


qBuffer = 1:150;
for j = 1:10
    fprintf('..');
    for i = 1:150
        q = ReadIMUQuaternion(serialObjIMU);
        
        qBuffer(i) = 100*q(4);
    end
    
    if (max(qBuffer) - min(qBuffer) >= 1)
        calibrated = false;
    else
        calibrated = true;
        break;
    end
end