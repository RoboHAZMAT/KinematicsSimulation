function KeyboardControl(RobotFigure, event, Robot)
%% ==========================Keyboard Control==============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 7, 2014
%
% Creates the control commands for the keyboard to be used during the
% simulaitons in real-time. Commands can be added for different key inputs.

% Parses the command key
command = event.Key;

% Gets the states of the Robot
states = guidata(RobotFigure);

% Advanced and Basic Controls
if (~states.advancedControl)
    if (strcmpi(command,'space')), states.run = 0; end;
    
else
    % Gets the step size
    step = states.step;
    
    % Gets the bases
    base = states.base;
    
    % Gets the arm states
    state = states.location;
    
    % Robot specific controls
    if (strcmpi(Robot.Name,'RoboHAZMAT'))
        
        % Right Arm commands
        if (strcmpi(command,'i')), state(1,1) = state(1,1) + step;
        elseif (strcmpi(command,'j')), state(2,1) = state(2,1) + step;
        elseif (strcmpi(command,'semicolon')), state(3,1) = state(3,1) + step;
        elseif (strcmpi(command,'k')), state(1,1) = state(1,1) - step;
        elseif (strcmpi(command,'l')), state(2,1) = state(2,1) - step;
        elseif (strcmpi(command,'slash')), state(3,1) = state(3,1) - step;
            
            % Left Arm commands
        elseif (strcmpi(command,'e')), state(1,2) = state(1,2) + step;
        elseif (strcmpi(command,'s')), state(2,2) = state(2,2) + step;
        elseif (strcmpi(command,'a')), state(3,2) = state(3,2) + step;
        elseif (strcmpi(command,'d')), state(1,2) = state(1,2) - step;
        elseif (strcmpi(command,'f')), state(2,2) = state(2,2) - step;
        elseif (strcmpi(command,'shift')), state(3,2) = state(3,2) - step;
        end;
        
    elseif (strcmpi(Robot.Name,'Mechatronic Arm'))
        
        % Mechatronic Arm commands
        if (strcmpi(command,'e')), state(1,1) = state(1,1) + step;
        elseif (strcmpi(command,'s')), state(2,1) = state(2,1) + step;
        elseif (strcmpi(command,'a')), state(3,1) = state(3,1) + step;
        elseif (strcmpi(command,'d')), state(1,1) = state(1,1) - step;
        elseif (strcmpi(command,'f')), state(2,1) = state(2,1) - step;
        elseif (strcmpi(command,'shift')), state(3,1) = state(3,1) - step;
        elseif (strcmpi(command,'equal')), states.gripper = 0; % Open
        elseif (strcmpi(command,'hyphen')), states.gripper = 1; % Close
        end
    end
    
    % General commands
    % Reset
    if (strcmpi(command,'return'))
        for i = 1:size(state,2), state(:,i) = states.start(:,i); end;
    % Exit Current Simulation
    elseif (strcmpi(command,'space')), states.run = 0;
    end;
    
    % Adjusts arm to zero
    for i = 1:size(state,2), state(:,i) = state(:,i) - base(:,i); end;
    
    % Bounds the states to the maximum arm length
    max = states.max;
    min = states.min;
    minmax = zeros(size(state,2));
    for i = 1:size(state,2)
        if (state(2,i) < min), state(2,:) = -state(2,i); minmax(i) = 1; end;
    end
    state(state > max) = max;
    state(state < min) = min;
    for i = 1:size(state,2)
        if (minmax), state(2,i) = -state(2,i); end;
    end
    
    % Final command location
    for i = 1:size(state,2)
        % Readjusts states to correct location
        state(:,i) = state(:,i) + base(:,i);
        % Sets the Robot states
        states.location(:,i) = state(:,i);
    end;
end

% Store data
guidata(RobotFigure,  states);