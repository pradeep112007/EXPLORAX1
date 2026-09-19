clear;
clc;
close all;

%% ==============================================
% PHASE 6 - LIVE CAMERA INPUT
%% ==============================================

% Show available cameras
cams = webcamlist;

disp("Available Cameras:");
disp(cams);

% Stop if no camera is detected
if isempty(cams)
    error("No camera detected. Install MATLAB Support Package for USB Webcams.");
end

% Connect to first camera
cam = webcam(1);

% Create window
fig = figure( ...
    'Name','Live Road Perception', ...
    'NumberTitle','off');

% Run live camera
while ishandle(fig)

    frame = snapshot(cam);

    imshow(frame);

    title( ...
        'LIVE CAMERA - ADAPTIVE PATH PLANNING SYSTEM', ...
        'FontSize',14, ...
        'FontWeight','bold');

    drawnow;

end

% Release camera
clear cam;