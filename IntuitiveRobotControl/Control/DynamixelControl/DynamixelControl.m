function DynamixelControl(dynamixel, serialObjArbotix, X, arm)
if (strcmpi(arm,'r'))
    X(1) = -X(1) + 125.5*pi/180;
    X(2) = -X(2) + (125.5+90)*pi/180;
    X(3) = X(3) + 125.5*pi/180;
    X(4) = round(mod((-X(4) + pi),2*pi)*4096/2*pi);
    X(5) = round(mod((-X(5) + pi),2*pi)*4096/2*pi);
    X(6) = round(mod((-X(6) + pi),2*pi)*4096/2*pi);
    for i = 1:3
        dynamixel{i}.setPositionEX(X(i),'rad');
    end
    for i = 4:6
        controlstring = '$' + '0' + int2str(i) + '!' + int2str(X(i)) + '*';
        serialObjArbotix.fwrite(controlstring);
    end
elseif (strcmpi(arm,'l'))
    X(1) = X(1) + 125.5*pi/180;
    X(2) = X(2) + (125.5+90)*pi/180;
    X(3) = X(3) + 125.5*pi/180;
    X(4) = round(mod((X(4) + pi),2*pi)*4096/2*pi);
    X(5) = round(mod((X(5) + pi),2*pi)*4096/2*pi);
    X(6) = round(mod((X(6) + pi),2*pi)*4096/2*pi);    
    for i = 1:3
        dynamixel{i}.setPositionEX(X(i),'rad');
    end
    for i = 4:6
        controlstring = '$' + '1' + int2str(i) + '!' + int2str(X(i)) + '*';
        serialObjArbotix.fwrite(controlstring);
    end
end