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

% Last Modified by GUIDE v2.5 11-May-2013 14:38:17

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



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

initialize_gui(hObject, handles, false);

% --------------------------------------------------------------------
%% User defined functions

function initialize_gui(fig_handle, handles, isreset)
% Initializing the GUI: setting default values
% also usable for resetting the GUI to default data

%% default values

% position / m
x_init = 2; 
y_init = 5;

% orientation angle / °
theta_init = -90;

% timestep / s
time_step_init = 1;


% set values to edit boxes in GUI
set(handles.ed_x_init, 'String', x_init);
set(handles.ed_y_init,  'String', y_init);
set(handles.ed_theta_init, 'String', theta_init);
set(handles.ed_timestep, 'String', time_step_init);

% Update handles structure
guidata(handles.figure1, handles);

