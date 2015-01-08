function [H]=DHTransforms(DH)
%
%   Return the homogeneous transforms for a kinematic chain
%   described by the DH parameters.
%
%   Input variables:
%       thetas(i)   -   rotation of link i for i=1...ndof
%       alphas(i)   -   twist of link i for i=1...ndof
%       disps(i)    -   displacement of link i for i=1...ndof
%       offsets(i)  -   offset of link i for i=1...ndof
%
%   Output variables
%       As(:,:,i)       -   is the 4x4xndof array of homogeneous transforms
%
thetas = DH.thetas;
alphas = DH.alphas;
disps = DH.disps;
offsets = DH.offsets;
DOF=length(thetas);
H=zeros(4,4,DOF);
Ht = zeros(4,4);

a=offsets(1);
d=disps(1);
st = sin(thetas(1));
ct = cos(thetas(1));
sa = sin(alphas(1));
ca = cos(alphas(1));
%
%   rotation
%
H(1,1,1)=ct;
H(1,2,1)=-st*ca;
H(1,3,1)=st*sa;
H(2,1,1)=st;
H(2,2,1)=ct*ca;
H(2,3,1)=-ct*sa;
% H(3,1,1)=0;
H(3,2,1)=sa;
H(3,3,1)=ca;
%
%   displacement
%
H(1,4,1)=a*ct;
H(2,4,1)=a*st;
H(3,4,1)=d;
H(4,4,1)=1;

for i=2:DOF
    a=offsets(i);
    d=disps(i);
    st = sin(thetas(i));
    ct = cos(thetas(i));
    sa = sin(alphas(i));
    ca = cos(alphas(i));
    %
    %   rotation
    %
    Ht(1,1)=ct;
    Ht(1,2)=-st*ca;
    Ht(1,3)=st*sa;
    Ht(2,1)=st;
    Ht(2,2)=ct*ca;
    Ht(2,3)=-ct*sa;
    % Ht(3,1)=0;
    Ht(3,2)=sa;
    Ht(3,3)=ca;
    %
    %   displacement
    %
    Ht(1,4)=a*ct;
    Ht(2,4)=a*st;
    Ht(3,4)=d;
    Ht(4,4)=1;
    H(:,:,i) = H(:,:,i-1)*Ht;
end
