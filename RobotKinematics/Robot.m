classdef Robot
    properties
        Name;
        KinematicChains;
        
    end
    
    methods
        function Robot = Robot(KCStruct)
            Robot.KinematicChains = (KCStruct.KC);
            Robot.Name = (KCStruct.name); 
        end
    end
end