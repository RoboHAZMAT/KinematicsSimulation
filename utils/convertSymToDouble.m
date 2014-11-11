function Y = convertSymToDouble(X,Z)



Y(:,:,1) = [ cos(th0),  0, -sin(th0),   0;
             sin(th0),  0,  cos(th0),   0;
                    0, -1,         0, d01;
                    0,  0,         0,   1];
 
Y(:,:,2) = [ cos(pi/2 + th1), 0,  sin(pi/2 + th1),   0;
             sin(pi/2 + th1), 0, -cos(pi/2 + th1),   0;
                           0, 1,                0, d12;
                           0, 0,                0,   1];
 
Y(:,:,3) = [ cos(pi/2 + th2), 0,  sin(pi/2 + th2),   0;
             sin(pi/2 + th2), 0, -cos(pi/2 + th2),   0;
                           0, 1,                0, d23;
                           0, 0,                0,   1];
 
Y(:,:,4) = [ cos(pi/2 + th3), 0,  sin(pi/2 + th3),   0;
             sin(pi/2 + th3), 0, -cos(pi/2 + th3),   0;
                           0, 1,                0, d34;
                           0, 0,                0,   1];
 
Y(:,:,5) = [ cos(th4),  0, -sin(th4),   0;
             sin(th4),  0,  cos(th4),   0;
                    0, -1,         0, d45;
                    0,  0,         0,   1];
 
Y(:,:,6) = [cos(th5), 0,  sin(th5),   0;
            sin(th5), 0, -cos(th5),   0;
                   0, 1,         0, d56;
                   0, 0,         0,   1];
 
Y(:,:,7) = [ cos(pi/2 + th6), 0,  sin(pi/2 + th6),   0;
             sin(pi/2 + th6), 0, -cos(pi/2 + th6),   0;
                           0, 1,                0, d67;
                           0, 0,                0,   1];
 
Y(:,:,8) = [ cos(th7), -sin(th7), 0,   0;
             sin(th7),  cos(th7), 0,   0;
                    0,         0, 1, d78;
                    0,         0, 0,   1];