function varargout = GUI_Mobile_Robot(varargin)
% GUI_MOBILE_ROBOT MATLAB code for GUI_Mobile_Robot.fig
%      GUI_MOBILE_ROBOT, by itself, creates a new GUI_MOBILE_ROBOT or raises the existing
%      singleton*.
%
%      H = GUI_MOBILE_ROBOT returns the handle to a new GUI_MOBILE_ROBOT or the handle to
%      the existing singleton*.
%
%      GUI_MOBILE_ROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_MOBILE_ROBOT.M with the given input arguments.
%
%      GUI_MOBILE_ROBOT('Property','Value',...) creates a new GUI_MOBILE_ROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_Mobile_Robot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_Mobile_Robot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_Mobile_Robot

% Last Modified by GUIDE v2.5 14-May-2013 14:32:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_Mobile_Robot_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_Mobile_Robot_OutputFcn, ...
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


% --- Executes just before GUI_Mobile_Robot is made visible.
function GUI_Mobile_Robot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_Mobile_Robot (see VARARGIN)

% initialize GUI
initialize_gui(hObject, handles, false);

% Choose default command line output for GUI_Mobile_Robot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_Mobile_Robot wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_Mobile_Robot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



%% EDIT BOXES

function ed_x_init_Callback(hObject, eventdata, handles)
% hObject    handle to ed_x_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_x_init as text
%        str2double(get(hObject,'String')) returns contents of ed_x_init as a double


% --- Executes during object creation, after setting all properties.
function ed_x_init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_x_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_y_init_Callback(hObject, eventdata, handles)
% hObject    handle to ed_y_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_y_init as text
%        str2double(get(hObject,'String')) returns contents of ed_y_init as a double


% --- Executes during object creation, after setting all properties.
function ed_y_init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_y_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ed_theta_init_Callback(hObject, eventdata, handles)
% hObject    handle to ed_theta_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_theta_init as text
%        str2double(get(hObject,'String')) returns contents of ed_theta_init as a double


% --- Executes during object creation, after setting all properties.
function ed_theta_init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_theta_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_timestep_Callback(hObject, eventdata, handles)
% hObject    handle to ed_timestep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_timestep as text
%        str2double(get(hObject,'String')) returns contents of ed_timestep as a double


% --- Executes during object creation, after setting all properties.
function ed_timestep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_timestep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in but_reset.
function but_reset_Callback(hObject, eventdata, handles)
% hObject    handle to but_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

initialize_gui(hObject, handles, false);

% --------------------------------------------------------------------
%% User defined functions

function initialize_gui(fig_handle, handles, isreset)
% Initializing the GUI: setting default values
% also usable for resetting the GUI to default data

% define some global settings
define_global_settings;
global timestep 

%% default values

% position / m
x_init = 2; 
y_init = 5;

% orientation angle / °
theta_init = -90;

% timestep / s
time_step_init = 1;

% speed / m/s
speed_init = 0.05;

% filename of default map (.bmp)
map_filename = 'workspace_map.bmp';

% room dimensions / m
room_width = 28.5;
room_height = 28.5;

% set values to edit boxes in GUI
set(handles.ed_x_init, 'String', x_init);
set(handles.ed_y_init,  'String', y_init);
set(handles.ed_theta_init, 'String', theta_init);
set(handles.ed_timestep, 'String', time_step_init);
set(handles.ed_speed, 'String', speed_init);
set(handles.ed_map_filename, 'String', map_filename);
set(handles.ed_room_width, 'String', room_width);
set(handles.ed_room_height, 'String', room_height);

% Update handles structure
guidata(handles.figure1, handles);



function ed_output_Callback(hObject, eventdata, handles)
% hObject    handle to ed_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_output as text
%        str2double(get(hObject,'String')) returns contents of ed_output as a double


% --- Executes during object creation, after setting all properties.
function ed_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in but_move_robot.
function but_move_robot_Callback(hObject, eventdata, handles)
% hObject    handle to but_move_robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% set values to edit boxes in GUI

% get current values from GUI
x_init = str2double(get(handles.ed_x_init, 'String'));
y_init = str2double(get(handles.ed_y_init, 'String'));
theta_init = str2double(get(handles.ed_theta_init, 'String'));
timestep = str2double(get(handles.ed_timestep, 'String'));
speed_init = str2double(get(handles.ed_speed, 'String'));
map_filename = get(handles.ed_map_filename, 'String');



function ed_speed_Callback(hObject, eventdata, handles)
% hObject    handle to ed_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_speed as text
%        str2double(get(hObject,'String')) returns contents of ed_speed as a double


% --- Executes during object creation, after setting all properties.
function ed_speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_map_filename_Callback(hObject, eventdata, handles)
% hObject    handle to ed_map_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_map_filename as text
%        str2double(get(hObject,'String')) returns contents of ed_map_filename as a double


% --- Executes during object creation, after setting all properties.
function ed_map_filename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_map_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in but_browse_map_file.
function but_browse_map_file_Callback(hObject, eventdata, handles)
% hObject    handle to but_browse_map_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename,filepath]=uigetfile('*.bmp');
set(handles.ed_map_filename, 'String', [filepath filename]);



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to ed_map_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_map_filename as text
%        str2double(get(hObject,'String')) returns contents of ed_map_filename as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_map_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in but_browse_map_file.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to but_browse_map_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in but_load_map.
function but_load_map_Callback(hObject, eventdata, handles)
% hObject    handle to but_load_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global workspace_data

% load the map
workspace_data.map_filename = get(handles.ed_map_filename, 'String');
workspace_data.map = imread(workspace_data.map_filename);

% plot the map
set(handles.ax_workspace,'Visible','On');
imshow(workspace_data.map,'Parent',handles.ax_workspace);
set(handles.ax_workspace,'NextPlot','add');

% map size / pixel
imgInfo=imfinfo(workspace_data.map_filename);
workspace_data.map_height=imgInfo.Height;
workspace_data.map_width=imgInfo.Width;


% --- Executes on button press in but_pick_points.
function but_pick_points_Callback(hObject, eventdata, handles)
% hObject    handle to but_pick_points (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global workspace_data
workspace_data.room_width = str2double(get(handles.ed_room_width, 'String'));
workspace_data.room_height = str2double(get(handles.ed_room_height, 'String'));


% collecting user defined point (by graphical clicking on axes)
% [x_sel,y_sel] = ginput
button=0;
x_sel = [];
y_sel = [];

% cur_axis_set = axis(handles.ax_workspace);
hold all
while (button~=3)&(button~=13)&(button~=27)
    try
        [x_pick,y_pick,button]=ginput(1);
    catch
        return;
    end
    if button==3 % right click -> no more point
        break;
    else
        x_sel = [x_sel;x_pick];    
        y_sel = [y_sel;y_pick]; 
    end
    plot(handles.ax_workspace,x_pick,y_pick,'color','g','Marker','+','LineWidth',1);
%     axis(cur_axis_set);
end
hold off

% transform to room coordinates (in m)
x_room = x_sel./workspace_data.map_width.*workspace_data.room_width;
y_room = y_sel./workspace_data.map_height.*workspace_data.room_height;

workspace_data.sel_points_px = [x_sel y_sel];
workspace_data.sel_points_m = [x_room y_room];


function ed_room_width_Callback(hObject, eventdata, handles)
% hObject    handle to ed_room_width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_room_width as text
%        str2double(get(hObject,'String')) returns contents of ed_room_width as a double


% --- Executes during object creation, after setting all properties.
function ed_room_width_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_room_width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_room_height_Callback(hObject, eventdata, handles)
% hObject    handle to ed_room_height (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_room_height as text
%        str2double(get(hObject,'String')) returns contents of ed_room_height as a double


% --- Executes during object creation, after setting all properties.
function ed_room_height_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_room_height (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
