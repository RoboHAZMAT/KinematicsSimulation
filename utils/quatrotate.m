% Rotate a vector r by quaternion q.  
% Also works on a matrix r if it is M-by-3
function rout = quatrotate(q,r)
  if(size(q,2) ~= 4), error('q must be 1-by-4'); end;
  if(size(r,2) ~= 3), error('r must be M-by-3'); end;
 
  % Normalize the quaternion
  %qin = q./( sqrt(sum(q.^2,2)) * ones(1,4));
  qin = q/norm(q);
 
  dcm = zeros(3,3);
  dcm(1,1) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
  dcm(1,2) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
  dcm(1,3) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
  dcm(2,1) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
  dcm(2,2) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
  dcm(2,3) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
  dcm(3,1) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
  dcm(3,2) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
  dcm(3,3) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
 
  rout = (dcm*r')';
 
end