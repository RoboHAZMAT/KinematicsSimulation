%% ================================Robot===================================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% November 15, 2014
%
% Class definition for the Robot Object. Composed of a set of kinematic
% chains and a given name for easy recognition.

classdef Robot
    properties
        Name;
        KinematicChains;
    end
    
    methods
        % Creates the Robot object from the Kinematic Chain struct.
        function Robot = Robot(KCStruct)
            Robot.KinematicChains = (KCStruct.KC);
            Robot.Name = (KCStruct.name); 
        end
    end
end