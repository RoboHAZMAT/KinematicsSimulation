classdef Dynamixel
    % The properties of the Dynamixel class
    properties
        address;      % Contains all of the needed packet addresses
        connection;   % Manages the connection to the servo
        property;     % Contains the servo motor properties
    end
    
    methods
        % Constructor for the Dynamixel class.
        function D = Dynamixel(varargin)
            if (nargin == 0), args = struct();
            else args = varargin{1};
            end
            % Sets the servo specific properties
            p = struct();
            if (isfield(args,'id')), p.ID = args.id;
            else p.ID = 0; end;
            if (isfield(args,'maxPosition'))
                p.MAX_POSITION = args.maxPosition;
            else p.MAX_POSITION = 4095;
            end
            if (isfield(args,'minPosition')),
                p.MIN_POSITION = args.minPosition;
            else p.MIN_POSITION = 0;
            end;
            if (isfield(args,'name')), p.NAME = args.name;
            else p.NAME = 'Unnammed'; end;
            D.property = p;
            
            % Communication Parameters
            c.CONNECTED = false;
            c.PORTNUM = 19;       % Com Port
            c.BAUDNUM = 1;       % Baudrate (1=1Mbps)
            D.connection = c;
            
            % General Dynamixel addresses
            a.ALARM_SHUTDOWN = 18;
            a.TORQUE_ENABLE = 24;
            a.D_GAIN = 26;
            a.I_GAIN = 27;
            a.P_GAIN = 28;
            a.GOAL_POSITION = 30;
            a.MOVING_SPEED = 32;
            a.TORQUE_LIMIT = 34;
            a.PRESENT_POSITION = 36;
            a.PRESENT_LOAD = 40;
            a.PRESENT_TEMP = 43;
            a.LOCK = 47;
            D.address = a;
            
            D = D.start();
            D.lock(true);
        end
        
        % Starts the connection to the motor and configures it.
        function this = start(this)
            result = calllib('dynamixel', 'dxl_initialize', ...
                this.connection.PORTNUM, this.connection.BAUDNUM);
            if result == 1;
                this.connection.CONNECTED = true;
                fprintf('Success: Connection Established!\n');
                pause(1)
                this.setCommand(this.address.TORQUE_ENABLE,1);
                this.setCommand(this.address.TORQUE_LIMIT,1023);
                this.setCommand(this.address.MOVING_SPEED,0);
            else
                this.connection.CONNECTED = false;
                fprintf('ERROR: Connection Failed!\n');
            end
        end
        
        %% Mehods for Control
        
        % Gets the names of the currently available statuses
        function address = getAddress(this)
            address = fieldnames(this.address);
            fprintf('Dynamixel Address Names:\n\n');
            for i = 1:length(address)
                fprintf('  %s: %i\n',address{i},this.address.(address{i}));
            end
        end
        
        % Generic setter command method
        function setCommand(this,address,value)
            calllib('dynamixel','dxl_write_word',this.property.ID,...
                address,value);
        end
        
        % Generic getter command method
        function value = getCommand(this,address)
            value = calllib('dynamixel','dxl_read_word',...
                this.property.ID,address);
        end
        
        % Sets the position to the given input. Includes limit checking.
        function setPosition(this,pos,type)
            % Allows a specified type of angle
            if (nargin > 2)
                if (strcmpi(type,'deg'))
                    pos = round(mod(pos,360)*4096/360);
                elseif (strcmpi(type,'rad'))
                    pos = round(mod(pos,(2*pi))*4096/(2*pi));
                end
            end
            % Checks that position is within the limits
            if (pos < this.property.MIN_POSITION)
                pos = this.property.MIN_POSITION;
                fprintf('\nWARNING: Position requested lower than limit\n');
            end
            if (pos > this.property.MAX_POSITION)
                pos = this.property.MAX_POSITION;
                fprintf('\nWARNING: Position requested greater than limit\n');
            end
            % Calls the library to set the position
            this.setCommand(this.address.GOAL_POSITION,pos);
        end
        
        % Gets the current position for the servo.
        function pos = getPosition(this, type)
            pos = int32(this.getCommand(this.address.PRESENT_POSITION));
            if (nargin > 1)
                if (strcmpi(type,'deg'))
                    pos = round(mod(pos,360)*360/4096);
                elseif (strcmpi(type,'rad'))
                    pos = round(mod(pos,(2*pi))*(2*pi)/4096);
                end
            end
        end
        
        % Sets the motor speed
        function setSpeed(this,speed)
            this.setCommand(this.address.MOVING_SPEED,speed);
        end
        
        % Gets the current speed for the servo.
        function speed = getSpeed(this)
            speed = int32(this.getCommand(this.address.MOVING_SPEED));
        end
        
        % Sets a new PID controller. -1 value to keep the same.
        function setPID(this,p,i,d)
            if (p < 0), p = this.getCommand(this.address.P_GAIN); end;
            if (i < 0), i = this.getCommand(this.address.I_GAIN); end;
            if (d < 0), d = this.getCommand(this.address.D_GAIN); end;
            this.setCommand(this.address.P_GAIN,p);
            this.setCommand(this.address.I_GAIN,i);
            this.setCommand(this.address.D_GAIN,d);
        end
        
        % Sets the motor to lock EEPROM. 1 to lock, 0 to unlock.
        function lock(this,value)
            if (value == true || value == false)
                this.setCommand(this.address.LOCK,value);
            end
        end
    end
end