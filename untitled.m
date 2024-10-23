% Clear workspace and reset image acquisition hardware
clear; close all; clc;
imaqreset;

% Step 1: Initialize the RealSense Camera
vid = videoinput('realsense', 1, 'RGB_640x480'); % RGB stream
src = getselectedsource(vid);
vid.FramesPerTrigger = 1;  % Capture one frame at a time
start(vid);  % Start the camera

% Step 2: Capture an initial frame and detect Harris corners
disp('Capturing initial frame...');
frame = getsnapshot(vid);  % Capture one frame
gray_frame = rgb2gray(frame);  % Convert to grayscale
corners = detectHarrisFeatures(gray_frame);  % Detect corners

% Step 3: Display the captured frame with detected corners
figure;
imshow(frame); hold on;
plot(corners.Location(:, 1), corners.Location(:, 2), 'r+', 'MarkerSize', 5);
title('Initial Frame with Detected Corners');

% Step 4: User selects two corner features
disp('Select the first corner feature from the image.');
[x1, y1] = ginput(1);  % User selects first feature
pattern1_current = [x1, y1];

disp('Select the second corner feature from the image.');
[x2, y2] = ginput(1);  % User selects second feature
pattern2_current = [x2, y2];

% Plot the selected features on the initial frame
hold on;
plot(x1, y1, 'ro', 'MarkerSize', 10, 'DisplayName', 'Feature 1 (Current)');
plot(x2, y2, 'go', 'MarkerSize', 10, 'DisplayName', 'Feature 2 (Current)');
legend;

% Step 5: User selects the desired positions for the two features
disp('Click on the desired position for Feature 1.');
[desired_x1, desired_y1] = ginput(1);  % Desired position for Feature 1
pattern1_desired = [desired_x1, desired_y1];

disp('Click on the desired position for Feature 2.');
[desired_x2, desired_y2] = ginput(1);  % Desired position for Feature 2
pattern2_desired = [desired_x2, desired_y2];

% Control parameters
lambda = 0.1;  % Control gain
tolerance = 2;  % Stop when error is within 2 pixels

% Step 6: Real-Time Visual Servoing Loop
disp('Starting real-time visual servoing...');
while true
    % Capture a new frame from the camera
    frame = getsnapshot(vid);
    gray_frame = rgb2gray(frame);
    corners = detectHarrisFeatures(gray_frame);  % Detect corners again

    % Compute error between current and desired positions
    error1 = pattern1_desired - pattern1_current;
    error2 = pattern2_desired - pattern2_current;

    % Check if the error is within the tolerance
    if norm(error1) < tolerance && norm(error2) < tolerance
        disp('Features aligned with desired positions.');
        break;
    end

    % Calculate movement based on the error
    movement1 = lambda * error1;
    movement2 = lambda * error2;

    % Update current positions
    pattern1_current = pattern1_current + movement1;
    pattern2_current = pattern2_current + movement2;

    % Display the new frame with updated positions
    clf;
    imshow(frame); hold on;
    plot(corners.Location(:, 1), corners.Location(:, 2), 'r+', 'MarkerSize', 5);
    plot(pattern1_current(1), pattern1_current(2), 'ro', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 1 (Current)');
    plot(pattern2_current(1), pattern2_current(2), 'go', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 2 (Current)');
    plot(pattern1_desired(1), pattern1_desired(2), 'rx', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 1 (Desired)');
    plot(pattern2_desired(1), pattern2_desired(2), 'gx', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 2 (Desired)');
    legend;
    title('Real-Time Visual Servoing');
    pause(0.1);  % Small pause for visualization

    % Display movement direction feedback in Command Window
    if movement1(1) < 0 || movement2(1) < 0
        disp('Move camera LEFT.');
    elseif movement1(1) > 0 || movement2(1) > 0
        disp('Move camera RIGHT.');
    end
    if movement1(2) < 0 || movement2(2) < 0
        disp('Move camera UP.');
    elseif movement1(2) > 0 || movement2(2) > 0
        disp('Move camera DOWN.');
    end
end

% Cleanup: Stop the camera and release resources
stop(vid);
disp('Visual servoing complete. Camera stopped.');
