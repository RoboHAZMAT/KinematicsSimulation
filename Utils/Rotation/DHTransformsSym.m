function [H]=DHTransformsSym(DH)
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
H = sym(H);
%
for i=1:DOF
    a=offsets(i);
    d=disps(i);
    %
    %   rotation
    %
    H(1,1,i)=cos(thetas(i));
    H(1,2,i)=-sin(thetas(i))*cos(alphas(i));
    H(1,3,i)=sin(thetas(i))*sin(alphas(i));
    H(2,1,i)=sin(thetas(i));
    H(2,2,i)=cos(thetas(i))*cos(alphas(i));
    H(2,3,i)=-cos(thetas(i))*sin(alphas(i));
    H(3,1,i)=0;
    H(3,2,i)=sin(alphas(i));
    H(3,3,i)=cos(alphas(i));
    %
    %   displacement
    %
    H(1,4,i)=a*cos(thetas(i));
    H(2,4,i)=a*sin(thetas(i));
    H(3,4,i)=d;
    H(4,4,i)=1;
end
    