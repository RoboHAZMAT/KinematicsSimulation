function DynamixelControl(dynamixel, serialObjArbotix, X, arm)
if (strcmpi(arm,'r'))
    X(1) = -X(1) + 125.5*pi/180;
    X(2) = -X(2) + (125.5+90)*pi/180;
    X(3) = X(3) + 125.5*pi/180;
    X(4) = round((-X(4) + pi)*4096/(2*pi));
    X(5) = round((X(5))*4096/(2*pi) + 2048);
    X(X < 0) = 0;
    X(X > 4095) = 4095;
    for i = 1:3
        dynamixel{i}.setPositionEX(X(i),'rad');
    end
    controlString1 = ['0', num2str(4),'!', num2str(X(4),'%04i')];
    controlString2 = ['0', num2str(5),'!', num2str(X(5),'%04i')];
    controlString = [controlString1,controlString2];
    fwrite(serialObjArbotix,controlString);
elseif (strcmpi(arm,'l'))
    X(1) = X(1) + 125.5*pi/180;
    X(2) = -X(2) + (125.5-90)*pi/180;
    X(3) = X(3) + 125.5*pi/180;
    X(4) = round((X(4) + pi)*4096/(2*pi));
    X(5) = round((X(5))*4096/(2*pi) + 2048);
    for i = 1:3
        dynamixel{i}.setPositionEX(X(i),'rad');
    end
    controlString1 = ['0', num2str(4),'!', num2str(X(4),'%04i')];
    controlString2 = ['0', num2str(5),'!', num2str(X(5),'%04i')];
    controlString = [controlString1,controlString2];
    %fwrite(serialObjArbotix,controlString);
end