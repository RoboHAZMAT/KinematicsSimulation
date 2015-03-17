function varargout = RobotSimulation(varargin)
%% ==========================Robot Simulation==============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% January 22, 2014
%
% Creates a GUI for the Robot Simulation. Allows the simulation to be more
% visual and accessible for everyone using it. Requires no programming
% knowledge as the 'RobotSim' wrapper class does. All simulation options
% and commands are controlled here at a higher level and therefore makes
% the robot more intuitive to use.

gui_Singleton = 1;
gui_State = struct('gui_Name', mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @RobotSimulation_OpeningFcn, ...
    'gui_OutputFcn',  @RobotSimulation_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before RobotSimulation is made visible.
function RobotSimulation_OpeningFcn(hObject, ~, handles, varargin)
% Choose default command line output for RobotSimulation
handles = StartUpGUI(hObject, handles);

% Update handles structure
guidata(hObject, handles);

function handles = StartUpGUI(hObject, handles)
handles.output = hObject;
handles.running = 0;

% Add the title screen Robot CAD model
axes(handles.robotCADAxes);
cla;
iptsetpref('ImshowBorder','tight')
imshow(imread('RoboHAZMAT_CAD.png'));

% Add color status block
set(handles.statusAxes, 'XTick', []);
set(handles.statusAxes, 'YTick', []);
set(handles.statusAxes,'Color',[1 1 0]);
SetSimulationControlText(handles,'','Select Simulation','Press Start','');

% --- Outputs from this function are returned to the command line.
function varargout = RobotSimulation_OutputFcn(~, ~, handles)
set(handles.robotAxes,'Visible','off')
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, ~, handles) %#ok<*DEFNU>
if(strcmpi(StopProgramDialog,'Yes'))
    set(handles.statusText,'String','Simulation Running...');
    set(handles.statusText,'String','Simulation Stopped.');
    set(handles.statusAxes,'Color',[1 0 0]);
    handles.run = 0; guidata(hObject, handles);
end;

% --- Executes on selection change in simulationSelectionMenu.
function simulationSelectionMenu_Callback(hObject, ~, handles)
handles.simulation = get(hObject,'Value');
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function simulationSelectionMenu_CreateFcn(hObject, ~, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.simulation = get(hObject,'Value');
guidata(hObject,handles);

% --- Executes on selection change in robotSelectionMenu.
function robotSelectionMenu_Callback(hObject, ~, handles)
handles.robotSystem = get(hObject,'Value');
guidata(hObject,handles);

if (handles.robotSystem == 1)
    optionStrings = {'Trajectory Tracking Simulation',...
        'Point Inverse Kinematics','Keyboard Control Simulation',...
        'IMU Controlled Arms (YPR)','Full  IMU Controlled Robot Arm',...
        'IMU Controlled Robot Head','RoboHAZMAT Control'};
elseif (handles.robotSystem == 2)
    optionStrings = {'Trajectory Tracking Simulation',...
        'Keyboard Control Simulation','Trajectory Tracking Arduino',...
        'Keyboard Control Arduino','IMU Controlled Mechatronic Arm'};
end

set(handles.simulationSelectionMenu,'String',optionStrings);


% --- Executes during object creation, after setting all properties.
function robotSelectionMenu_CreateFcn(hObject, ~, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.robotSystem = get(hObject,'Value');
guidata(hObject,handles);


% --- Executes on button press in startButton.
function startButton_Callback(hObject, ~, handles)
cla(handles.robotCADAxes);
set(handles.statusAxes,'Color',[0 1 0]);
axes(handles.robotAxes);
set(handles.robotAxes,'Visible','on')
set(handles.statusText,'String','Simulation Running...');

if (~handles.running)
    handles.running = 1; guidata(hObject,handles);
    switch handles.robotSystem
        case 1
            RobotSim(handles.simulation);
        case 2
            RobotSim(handles.simulation + 10);
    end
    handles = guidata(hObject);
    handles.running = 0; guidata(hObject,handles);
end


% --- Executes on button press in zoomToggleButton.
function zoomToggleButton_Callback(hObject, ~, handles)
zoomAxes = get(hObject,'Value');
if (zoomAxes)
    zoom on;
    set(handles.rotateToggleButton, 'Value', 0);
    set(handles.panToggleButton, 'Value', 0);
else zoom off;
end


% --- Executes on button press in panToggleButton.
function panToggleButton_Callback(hObject, ~, handles)
panAxes = get(hObject,'Value');
if (panAxes)
    pan on;
    set(handles.rotateToggleButton, 'Value', 0);
    set(handles.zoomToggleButton, 'Value', 0);
else pan off;
end


% --- Executes on button press in rotateToggleButton.
function rotateToggleButton_Callback(hObject, ~, handles)
rotateAxes = get(hObject,'Value');
if (rotateAxes)
    rotate3d on;
    set(handles.panToggleButton, 'Value', 0);
    set(handles.zoomToggleButton, 'Value', 0);
else rotate3d off; ReleaseFocus(handles.robotSimulationFigure);
end


% --- Executes on button press in resetButton.
function resetButton_Callback(~, ~, ~)
% hObject    handle to resetButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function ReleaseFocus(fig)
set(findobj(fig, 'Type', 'uicontrol'), 'Enable', 'off');
drawnow;
set(findobj(fig, 'Type', 'uicontrol'), 'Enable', 'on');
drawnow;
