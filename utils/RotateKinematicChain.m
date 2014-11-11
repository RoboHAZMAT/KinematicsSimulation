function KC = RotateKinematicChain(KC, X)

KC.DHParams.thetas = (KC.thetas.thi + X);

% Homogeneous transformations
KC.DHTransforms.H = double(DHTransforms(KC.DHParams,false));

% Transform each point in the global frame
for i = 1:KC.DOF
    % the points in Global Coordinates
    KC.DHTransforms.HG = KC.DHTransforms.HGo;
    
    for j = 1:i
        KC.DHTransforms.HG = KC.DHTransforms.HG*KC.DHTransforms.H(:,:,j);
    end
    KC.points.pG(:,i) = KC.DHTransforms.HG*KC.points.p(:,i);
end