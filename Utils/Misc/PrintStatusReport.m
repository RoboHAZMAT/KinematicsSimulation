function status = PrintStatusReport(KC, X, status)

X = X*180/pi;
fprintf(1, repmat('\b',1,status.count));
status.count = fprintf('  Base Yaw   Base Pitch   Elbow Pitch   Wrist Pitch\n');
status.count = status.count + fprintf('%9.3f%12.3f%14.3f%14.3f\n',...
    X(1),X(2),X(3),X(4));

status.count = status.count + fprintf('=====================================================\n');
for i = 1:size(status.point,1);
    status.count = status.count + fprintf(' %i:   X: %7.2f        Y: %7.2f       Z: %7.2f\n',...
        status.point(i),KC.points.kPG(1,status.point(i))*100,...
        KC.points.kPG(2,status.point(i))*100,...
        KC.points.kPG(3,status.point(i))*100);
end