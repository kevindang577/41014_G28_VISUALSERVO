% Clear workspace and command window
clear; clc;

% Declare global variables
global img pointTracker pointsInitialized userSelectedPoints selectionMode cameraAxes selectedCentroid
global moveX moveY moveZ ur3Robot ur3Axes % Added global variables for UR3 movement

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

% Load camera parameters from calibration 'cameraParams.mat'
try
    load('cameraParams.mat', 'cameraParams');
    if ~exist('cameraParams', 'var')
        error('Variable ''cameraParams'' not found in cameraParams.mat.');
    end
catch
    error(['Unable to load camera parameters. Please ensure that ' ...
        'cameraParams.mat exists and contains the variable ''cameraParams''.']);
end

% Initialize 'img' by capturing an initial frame
frames = pipeline.wait_for_frames();
color_frame = frames.get_color_frame();

% Check if frame is valid
if isempty(color_frame)
    error('No frames received from the camera.');
end

% Convert the color frame to MATLAB image
img = color_frame.get_data();
img = permute(reshape(img', [3, color_frame.get_width(), color_frame.get_height()]), [3, 2, 1]);
img = im2uint8(img);

% Set up the figure and layout
hFigure = figure('Name', 'Visual Servoing with Intel RealSense D435', 'NumberTitle', 'off', 'MenuBar', 'none');

% Create a panel for the camera output
cameraPanel = uipanel('Parent', hFigure, 'Units', 'normalized', 'Position', [0 0 0.75 1]);

% Create a panel for the GUI controls
controlPanel = uipanel('Parent', hFigure, 'Units', 'normalized', 'Position', [0.75 0 0.25 1]);

% Create axes for displaying the camera image
cameraAxes = axes('Parent', cameraPanel, 'Units', 'normalized', 'Position', [0 0 1 1]);

% Set the cursor to a crosshair
set(hFigure, 'Pointer', 'crosshair');

% Variables to store user-selected points and tracking status
userSelectedPoints = [];
pointsInitialized = false;
selectionMode = ''; % '3-points' or '4-points'
selectedCentroid = [];

% Initialize point tracker
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% --- GUI Controls ---

% Button for Selection Mode (3 Corners)
btn3Corners = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Select (Triangle)', ...
    'Units', 'normalized', 'Position', [0.1 0.8 0.8 0.1], 'Callback', @btn3CornersCallback, 'FontSize', 12);

% Button for Selection Mode (4 Corners)
btn4Corners = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Select (Square)', ...
    'Units', 'normalized', 'Position', [0.1 0.65 0.8 0.1], 'Callback', @btn4CornersCallback, 'FontSize', 12);

% Button to show simulation
btnSimulation = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Open UR3 Simulation', ...
    'Units', 'normalized', 'Position', [0.1 0.5 0.8 0.1], 'Callback', @btnSimulationCallback, 'FontSize', 12);

% Button to Exit Selection Mode
btnExitSelection = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Exit Selection Mode', ...
    'Units', 'normalized', 'Position', [0.1 0.35 0.8 0.1], 'Callback', @btnExitSelectionCallback, 'FontSize', 12);

% Button to Exit Program
btnExitProgram = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Exit Program', ...
    'Units', 'normalized', 'Position', [0.1 0.2 0.8 0.1], 'Callback', @btnExitProgramCallback, 'FontSize', 12);

% --- Main Loop ---
% --- Main Loop ---
try
    % Declare global variables for real-time updates
    global moveX moveY moveZ

    while ishandle(hFigure)
        % Wait for the next set of frames (color frame)
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

        % Convert to grayscale
        grayImage = im2gray(img);

        % If points have been selected and initialized, track them
        if pointsInitialized
            % Track the points
            [trackedPoints, validity] = pointTracker(img);

            % If enough points are valid, estimate transformation
            if sum(validity) >= size(userSelectedPoints, 1)
                % Select valid points
                oldPointsValid = userSelectedPoints(validity, :);
                newPointsValid = trackedPoints(validity, :);

                % Compute the current centroid of the tracked points
                currentCentroid = mean(newPointsValid, 1);

                % Compute the displacement between current and initial centroids
                displacement = currentCentroid - selectedCentroid;

                % Map the displacement to movement directions
                moveX = displacement(1); % Right (+), Left (-)
                moveY = -displacement(2); % Up (+), Down (-)

                % Estimate scale change between initial and current points
                initialDistances = pdist2(userSelectedPoints, selectedCentroid);
                currentDistances = pdist2(newPointsValid, currentCentroid);
                scaleChange = mean(currentDistances) / mean(initialDistances);

                % Approximate Z-direction movement based on scale change
                moveZ = scaleChange - 1; % Forward (-), Backward (+)

                % Call updateUR3Position directly here to apply movement to UR3
                updateUR3Position();

                % Display movement direction
                imshow(img, 'Parent', cameraAxes);
                hold(cameraAxes, 'on');
                plot(cameraAxes, newPointsValid(:,1), newPointsValid(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
                plot(cameraAxes, currentCentroid(1), currentCentroid(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
                hold(cameraAxes, 'off');
                drawnow;

                % Update points
                userSelectedPoints = newPointsValid;
                setPoints(pointTracker, userSelectedPoints);
            else
                disp('Tracking lost. Please reselect points.');
                pointsInitialized = false;
                release(pointTracker);
            end
        else
            % Detect corners
            corners = detectHarrisFeatures(grayImage);
            imshow(img, 'Parent', cameraAxes);
            hold(cameraAxes, 'on');
            strongestCorners = corners.selectStrongest(200);
            cornerCoords = strongestCorners.Location;
            plot(cameraAxes, cornerCoords(:,1), cornerCoords(:,2), 'o', 'Color', 'blue');
            hold(cameraAxes, 'off');
            drawnow;
        end
    end
catch ME
    disp(['Error: ' ME.message]);
end



% Clean up
pipeline.stop();
close all;

% --- Callback Functions ---

% Callback for Selection Mode (3 Corners) Button
function btn3CornersCallback(~, ~)
    global img pointTracker pointsInitialized userSelectedPoints selectionMode cameraAxes selectedCentroid
    selectionMode = '3-points';
    disp('Please select 3 points.');

    % Let the user select points
    [x, y] = getPointsFromDetectedCorners(img, 3, cameraAxes);
    userSelectedPoints = [x, y];
    selectedCentroid = mean(userSelectedPoints, 1);

    % Initialize the point tracker with selected points
    release(pointTracker);
    initialize(pointTracker, userSelectedPoints, img);
    pointsInitialized = true;
end

% Callback for Selection Mode (4 Corners) Button
function btn4CornersCallback(~, ~)
    global img pointTracker pointsInitialized userSelectedPoints selectionMode cameraAxes selectedCentroid
    selectionMode = '4-points';
    disp('Please select 4 points.');

    % Let the user select points
    [x, y] = getPointsFromDetectedCorners(img, 4, cameraAxes);
    userSelectedPoints = [x, y];
    selectedCentroid = mean(userSelectedPoints, 1);

    % Initialize the point tracker with selected points
    release(pointTracker);
    initialize(pointTracker, userSelectedPoints, img);
    pointsInitialized = true;
end

% Callback for Open UR3 Simulation Button
function btnSimulationCallback(~, ~)
    global ur3Robot ur3Axes positionData

    % Create a new figure for the UR3 simulation
    simFigure = figure('Name', 'UR3 Simulation', 'NumberTitle', 'off', 'MenuBar', 'none', 'Position', [100, 100, 800, 600]);
    
    % Set up axes for UR3 simulation
    ur3Axes = axes('Parent', simFigure, 'Units', 'normalized', 'Position', [0.05, 0.05, 0.9, 0.9]);
    hold(ur3Axes, 'on');
    axis(ur3Axes, [-1.5 1.5 -1.5 1.5 0 1.5]);
    xlabel(ur3Axes, 'X');
    ylabel(ur3Axes, 'Y');
    zlabel(ur3Axes, 'Z');
    grid(ur3Axes, 'on');
    view(ur3Axes, 3);

    % Initialize UR3 robot in this environment
    ur3Robot = UR3(transl(0, 0, 0.75)); % Adjust initial base position as needed

    % Plot the UR3 robot in the current axes
    axes(ur3Axes);  % Set the created axes as the active axes
    ur3Robot.model.plot(zeros(1, 6), 'workspace', [-1 1 -1 1 0 1]);

    % Initialize the initial joint positions (positionData) to zeros
    positionData = zeros(1, ur3Robot.model.n);

    % Clean up resources when the figure is closed
    set(simFigure, 'CloseRequestFcn', @(src, ~) delete(src));
end

function updateUR3Position()
    global ur3Robot positionData moveX moveY moveZ

    % Early exit if moveX, moveY, and moveZ values have not been updated
    if isempty(ur3Robot) || all([moveX, moveY, moveZ] == 0)
        return;
    end

    % Set a higher control gain to improve responsiveness
    gain = 0.005; % Increase gain to make movements more pronounced

    % Calculate desired end-effector velocity with increased scaling for better visibility
    desiredVelocity = gain * [moveX; moveY; moveZ; 0; 0; 0]; % [vx; vy; vz; 0; 0; 0] (no rotational components)

    % Calculate the Jacobian in the current configuration
    J = ur3Robot.model.jacobe(positionData); % Current configuration Jacobian in end-effector frame

    % Compute joint velocities using Jacobian transpose for velocity control
    jointVelocities = J' * desiredVelocity;

    % Increase the integration time step for more noticeable movements
    dt = 0.2; % Increased time step to make the movement more substantial
    newJointPositions = positionData + jointVelocities' * dt;

    % Animate the UR3 robot in the new configuration
    ur3Robot.model.animate(newJointPositions);

    % Update the current joint positions
    positionData = newJointPositions;
end


% Function to stop and delete the timer on figure close
function closeSimulation(figHandle, timerHandle)
    stop(timerHandle);
    delete(timerHandle);
    delete(figHandle);
end



% Callback for Exit Selection Mode Button
function btnExitSelectionCallback(~, ~)
    global pointsInitialized
    disp('Exiting selection mode.');
    pointsInitialized = false;
end

% Callback for Exit Program Button
function btnExitProgramCallback(~, ~)
    disp('Exiting program.');
    close all;
end

% Function to select points from detected corners
function [x, y] = getPointsFromDetectedCorners(img, numPoints, cameraAxes)
    grayImage = im2gray(img);
    corners = detectHarrisFeatures(grayImage);
    imshow(img, 'Parent', cameraAxes);
    hold(cameraAxes, 'on');
    strongestCorners = corners.selectStrongest(200);
    cornerCoords = strongestCorners.Location;
    plot(cameraAxes, cornerCoords(:,1), cornerCoords(:,2), 'o', 'Color', 'blue');
    hold(cameraAxes, 'off');
    drawnow;
    [x, y] = ginput(numPoints);
end