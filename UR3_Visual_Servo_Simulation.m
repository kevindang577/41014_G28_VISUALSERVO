clc;
clear;
close all;

%% Add necessary paths
% Add paths to Peter Corke's Robotics Toolbox and Machine Vision Toolbox
% Adjust the paths as needed for your system
addpath('path_to_robotics_toolbox');
addpath('path_to_machine_vision_toolbox');

% Add path to the RealSense MATLAB wrappers
% Replace 'path_to_realsense_matlab_wrappers' with the actual path
addpath('path_to_realsense_matlab_wrappers');

%% Initialize Intel RealSense D455 Camera
% Create a pipeline object to manage streaming
pipeline = realsense.pipeline();

% Configure the pipeline to stream color frames
config = realsense.config();
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);

% Start streaming
profile = pipeline.start(config);

% Get device information (optional)
device = profile.get_device();
name = device.get_info(realsense.camera_info.name);
serial = device.get_info(realsense.camera_info.serial_number);
disp(['Connected to: ' name ' ' serial]);

%% Set up the environment with table and UR3e robot
figure('Name', 'UR3e Visual Servoing', 'NumberTitle', 'off');

% Load the table model
Table = PlaceObject('counter.ply');
Table_vertices = get(Table, 'Vertices');
transformedVerticesT = [0.5 * Table_vertices, ones(size(Table_vertices, 1), 1)] * troty(-pi/2)' * transl(0, 0, 0)';
set(Table, 'Vertices', transformedVerticesT(:, 1:3));

% Create the UR3e robot
robot = UR3(); % Assuming UR3e is similar to UR3 in the toolbox
robot.model.base = transl(0, 0, 0.75); % Set the robot base position

% Set up the 3D plot for the robot and environment
hold on;
workspace = [-1.5, 1.5, -1.5, 1.5, 0, 2];
robot.model.plot(zeros(1, robot.model.n), 'workspace', workspace);
view(3);
axis equal;
grid on;

% Create a wall to display the live video feed
wallX = [-1.5, 1.5; -1.5, 1.5];
wallY = [1.5, 1.5; 1.5, 1.5];
wallZ = [0, 0; 2, 2];
wall = surf(wallX, wallY, wallZ, 'CData', zeros(2,2,3), 'FaceColor', 'texturemap');

% Initialize trace for visualization
traceHandle = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5);

% Initialize variables for end-effector path
tracePoints = []; % Store the path for continuous line update

% Define end-effector orientation for tracing (pointing down)
targetOri = trotx(-pi/2) * troty(pi/2);

%% Main Loop for Visual Servoing
try
    while true
        %% Capture a frame from the camera
        frames = pipeline.wait_for_frames();
        color_frame = frames.get_color_frame();

        % If no frame is available, continue
        if isempty(color_frame)
            continue;
        end

        % Convert the color frame to MATLAB image
        img = color_frame.get_data();
        img = permute(reshape(img', [3, color_frame.get_width(), color_frame.get_height()]), [3, 2, 1]);
        img = im2uint8(img);

        % Display the live video feed on the wall
        set(wall, 'CData', flipud(img)); % Flip the image vertically for correct display
        drawnow;

        %% Process the frame to detect corners
        grayImage = rgb2gray(img);

        % Apply Gaussian blurring to reduce noise
        grayImage = imgaussfilt(grayImage, 2); % Adjust the sigma value as needed

        % Harris corner detection
        corners = detectHarrisFeatures(grayImage, 'MinQuality', 0.1, 'FilterSize', 5);
        cornerPoints = corners.Location; % Extract corner coordinates

        % If not enough corners are detected, skip this frame
        if size(cornerPoints, 1) < 3
            disp('Not enough corners detected.');
            continue;
        end

        % Compute the centroid of the detected corners
        centroid = mean(cornerPoints, 1);

        %% Convert centroid from image coordinates to robot's workspace coordinates
        % Assuming the wall is at Y = 1.5 in the robot's workspace
        % Map image coordinates to wall coordinates
        imageSize = size(grayImage);
        wallWidth = 3.0; % Width of the wall in meters (from X = -1.5 to X = 1.5)
        wallHeight = 2.0; % Height of the wall (from Z = 0 to Z = 2)

        % Calculate scaling factors
        scaleX = wallWidth / imageSize(2);
        scaleZ = wallHeight / imageSize(1);

        % Map centroid to world coordinates on the wall
        centroidWorldX = -1.5 + centroid(1) * scaleX;
        centroidWorldY = 1.5; % Wall is at Y = 1.5
        centroidWorldZ = centroid(2) * scaleZ;

        targetPoint = [centroidWorldX, centroidWorldY - 0.3, centroidWorldZ]; % Adjust Y to be slightly in front of the wall

        %% Move UR3e to align the end-effector with the centroid
        moveUR3ToPoint(robot, targetPoint, targetOri);

        % Append the current target position to the trace
        tracePoints = [tracePoints; targetPoint];

        % Update the 3D trace on the plot
        set(traceHandle, 'XData', tracePoints(:,1), 'YData', tracePoints(:,2), 'ZData', tracePoints(:,3));
        drawnow;

        pause(0.05); % Adjust for speed of tracing
    end
catch ME
    disp(['Error: ' ME.message]);
end

%% Clean up
pipeline.stop();
close all;

%% Function Definitions

function moveUR3ToPoint(robot, targetPoint, targetOri)
    % Move UR3 robot to a specific point with specified orientation
    [matrixSignal, qMatrix] = FindqMatrix(robot, targetPoint, targetOri);
    if matrixSignal
        run(robot, qMatrix); % Run movement if a valid path is found
    else
        disp('Failed to find a valid path for tracing.');
    end
end

function [result, qMatrix] = FindqMatrix(robot, targetPos, targetOri)
    % Calculate joint configurations to reach the target position
    qMatrix = [];
    Q_destination = SE3(targetPos) * SE3(targetOri);
    initialGuess = robot.model.getpos();

    % Set options for inverse kinematics with relaxed constraints
    ikine_options = {'tol', 1e-4, 'lambda', 0.5, 'lambdamin', 0.01, 'ilimit', 1000, 'mask', [1 1 1 0 0 0]};
    
    Q = robot.model.ikcon(Q_destination.T, initialGuess, ikine_options{:});
    if ~isempty(Q)
        result = 1;
        qMatrix = jtraj(initialGuess, Q, 50); % Generate smooth trajectory
    else
        result = 0;
    end
end

function run(robot, qMatrix)
    % Run the UR3 through the calculated joint trajectory and plot a trace
    if ~isempty(qMatrix)
        for i = 1:size(qMatrix, 1)
            robot.model.animate(qMatrix(i, :));
            drawnow;
        end
    end
end

function h = PlaceObject(name)
    % Helper function to load and place objects in the environment
    [f, v, data] = plyread(name, 'tri');
    h = trisurf(f, v(:,1), v(:,2), v(:,3), 'FaceVertexCData', data.vertex.red/255, 'EdgeColor', 'none', 'FaceColor', 'interp');
end
