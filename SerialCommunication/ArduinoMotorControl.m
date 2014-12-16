i = 0;
while(1)
    if (i >= 180)
        forwards = 0;
    elseif (i <= 0)
        forwards = 1;
    end
    a.servoWrite(9,round(i/3+60));
    a.servoWrite(10,i);
    if (forwards == 1)
        i = i + 1;
    elseif (forwards == 0)
        i = i - 1;
    end
    pause(0.01)
end