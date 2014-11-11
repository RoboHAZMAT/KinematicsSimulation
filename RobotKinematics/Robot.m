classdef Robot
    properties
        Name = 'RoboHAZMAT';
        KinematicChains;
        
    end
    
    methods
        function Robot = Robot(KC)
            Robot.KinematicChains = (KC);
        end
    end
end