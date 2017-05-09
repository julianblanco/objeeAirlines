function varargout = objeeAirlines(varargin)
% OBJEEAIRLINES MATLAB code for objeeAirlines.fig
%      OBJEEAIRLINES, by itself, creates a new OBJEEAIRLINES or raises the existing
%      singleton*.
%
%      H = OBJEEAIRLINES returns the handle to a new OBJEEAIRLINES or the handle to
%      the existing singleton*.
%
%      OBJEEAIRLINES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OBJEEAIRLINES.M with the given input arguments.
%
%      OBJEEAIRLINES('Property','Value',...) creates a new OBJEEAIRLINES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before objeeAirlines_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to objeeAirlines_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help objeeAirlines

% Last Modified by GUIDE v2.5 07-Apr-2016 15:13:02

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @objeeAirlines_OpeningFcn, ...
                   'gui_OutputFcn',  @objeeAirlines_OutputFcn, ...
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


% --- Executes just before objeeAirlines is made visible.
function objeeAirlines_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to objeeAirlines (see VARARGIN)

% Choose default command line output for objeeAirlines
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes objeeAirlines wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = objeeAirlines_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function NewLatbox_Callback(hObject, eventdata, handles)
% hObject    handle to NewLatbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NewLatbox as text
%        str2double(get(hObject,'String')) returns contents of NewLatbox as a double


% --- Executes during object creation, after setting all properties.
function NewLatbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NewLatbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function NewLongbox_Callback(hObject, eventdata, handles)
% hObject    handle to NewLongbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of NewLongbox as text
%        str2double(get(hObject,'String')) returns contents of NewLongbox as a double


% --- Executes during object creation, after setting all properties.
function NewLongbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to NewLongbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in AddWP.
function AddWP_Callback(hObject, eventdata, handles)
% hObject    handle to AddWP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function textWP_Callback(hObject, eventdata, handles)
% hObject    handle to textWP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of textWP as text
%        str2double(get(hObject,'String')) returns contents of textWP as a double


% --- Executes during object creation, after setting all properties.
function textWP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textWP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in newWP.
function newWP_Callback(hObject, eventdata, handles)
% hObject    handle to newWP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
