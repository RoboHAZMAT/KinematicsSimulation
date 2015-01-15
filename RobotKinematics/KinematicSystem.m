%% ===========================Kinematic System=============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% November 15, 2014
%
% Class definition for the Kinematic System. Composed of a given name for 
% easy recognition and all of the necessary describing features of a 
% kinematic system as defined in the '{}Kinematics.m' files.

classdef KinematicSystem
    properties
        Name;           % Name of the kinematic system
        Field;
        DOF;            % Number of Degrees of Freedom
        points;         % Struct of points in respective frames and global
                        % o: origin, p: in frame, pG: global
        dists;          % Distances between points
        thetas;         % Thetas to rotate each DOF
        DHParams;       % DH parameters for the kinematic chain
        symbolic;       % Holds the symbolic variables
        optimization;   % Optimization parameters, weights and bounds
    end
    
    methods
        % Parse the Kinematic System struct into a Kinematic System object.
        function KS = KinematicSystem(KSStruct)            
            KS.Name = KSStruct.Name;
            KS.Field = KSStruct.Field;
            if (~strcmp('Empty',KS.Name))
                fprintf('Kinematic System: %s\nField: %s\n\n',KS.Name,...
                    KS.Field);
            end
            KS.DOF = KSStruct.DOF;
            KS.points = KSStruct.pts;
            KS.dists = KSStruct.d;
            KS.optimization = KSStruct.opt;
            KS.thetas = KSStruct.th;
            KS.DHParams = KSStruct.DH;
            KS.symbolic = KSStruct.symbs;
        end
    end
end