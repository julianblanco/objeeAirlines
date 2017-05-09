function varargout = jackgui(varargin)
% JACKGUI MATLAB code for jackgui.fig
%      JACKGUI, by itself, creates a new JACKGUI or raises the existing
%      singleton*.
%
%      H = JACKGUI returns the handle to a new JACKGUI or the handle to
%      the existing singleton*.
%
%      JACKGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JACKGUI.M with the given input arguments.
%
%      JACKGUI('Property','Value',...) creates a new JACKGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before jackgui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to jackgui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help jackgui

% Last Modified by GUIDE v2.5 11-Mar-2015 23:52:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jackgui_OpeningFcn, ...
                   'gui_OutputFcn',  @jackgui_OutputFcn, ...
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


% --- Executes just before jackgui is made visible.
function jackgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to jackgui (see VARARGIN)

% Choose default command line output for jackgui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes jackgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = jackgui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
