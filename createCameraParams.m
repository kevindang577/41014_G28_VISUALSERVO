% Clear workspace and close figures
clear;
clc;
close all;

%% Add the RealSense SDK to MATLAB path
% Replace 'path_to_realsense_sdk' with the actual path
addpath('path_to_realsense_sdk');

%% Initialize the RealSense context
ctx = realsense.context();

% Get the connected devices
devs = ctx.query_devices();

% Check if any device is connected
if devs.size() == 0
    error('No Intel RealSense devices were found.');
end

% Get the first device
dev = devs{1};

% Get the sensors
sensors = dev.query_sensors();

% Initialize variables
color_sensor = [];
depth_sensor = [];

% Loop through sensors to find the color sensor
for i = 1:length(sensors)
    sensor = sensors{i};
    sensor_name = sensor.get_info(realsense.camera_info.name);
    if strcmp(sensor_name, 'RGB Camera')
        color_sensor = sensor;
    elseif strcmp(sensor_name, 'Stereo Module')
        depth_sensor = sensor;
    end
end

if isempty(color_sensor)
    error('Color sensor not found.');
end

% Get the stream profiles from the color sensor
stream_profiles = color_sensor.get_stream_profiles();

% Find the video stream profile with desired format and resolution
video_stream_profile = [];
for i = 1:length(stream_profiles)
    sp = stream_profiles{i};
    if sp.stream_type() == realsense.stream.color && ...
       sp.format() == realsense.format.rgb8
       
        % Cast to video_stream_profile
        if ismethod(sp, 'as_video_stream_profile')
            vsp = sp.as_video_stream_profile();
        else
            % Alternative casting method
            vsp = realsense.video_stream_profile(sp.handle);
        end
        
        % Check resolution
        if vsp.width() == 640 && vsp.height() == 480
            video_stream_profile = vsp;
            break;
        end
    end
end

if isempty(video_stream_profile)
    error('Matching video stream profile not found.');
end

% Get the intrinsics
intrinsics = video_stream_profile.get_intrinsics();

% Extract intrinsic parameters
fx = intrinsics.fx; % Focal length in pixels along x-axis
fy = intrinsics.fy; % Focal length in pixels along y-axis
ppx = intrinsics.ppx; % Principal point x-coordinate in pixels
ppy = intrinsics.ppy; % Principal point y-coordinate in pixels
distortion_coeffs = intrinsics.coeffs; % Distortion coefficients

% Image size
imageSize = [intrinsics.height, intrinsics.width];

% Focal lengths
focalLength = [fx, fy];

% Principal point
principalPoint = [ppx, ppy];

% Distortion coefficients mapping
% The RealSense SDK provides coefficients in the order:
% [k1, k2, p1, p2, k3]
% MATLAB's cameraParameters expects distortion coefficients accordingly
radialDistortion = [distortion_coeffs(1), distortion_coeffs(2), distortion_coeffs(5)];
tangentialDistortion = [distortion_coeffs(3), distortion_coeffs(4)];

% Create cameraParameters object
cameraParams = cameraParameters(...
    'IntrinsicMatrix', [fx, 0, 0; 0, fy, 0; ppx, ppy, 1]', ...
    'RadialDistortion', radialDistortion, ...
    'TangentialDistortion', tangentialDistortion, ...
    'ImageSize', imageSize);

% Save the cameraParams variable to cameraParams.mat
save('cameraParams.mat', 'cameraParams');

disp('cameraParams.mat has been saved successfully.');
