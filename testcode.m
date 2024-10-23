

% Clear workspace and close all figures
clear; close all; clc;

% Step 1: Load the uploaded images
img_close = imread('C:\Users\Kevin\Desktop\Major Project - Sensors & Control\close fov.png');  % Close-up image
img_medium = imread('C:\Users\Kevin\Desktop\Major Project - Sensors & Control\medium fov.png'); % Medium distance image
img_far = imread('C:\Users\Kevin\Desktop\Major Project - Sensors & Control\far fov.png');      % Far distance image

% Step 2: Convert images to grayscale for corner detection
gray_close = rgb2gray(img_close);
gray_medium = rgb2gray(img_medium);
gray_far = rgb2gray(img_far);

% Step 3: Detect Harris corners in the images
corners_close = detectHarrisFeatures(gray_close);
corners_medium = detectHarrisFeatures(gray_medium);
corners_far = detectHarrisFeatures(gray_far);

% Step 4: Display the images with detected corners
figure;

% Close FOV Image Plot
subplot(1, 3, 1); 
imshow(img_close); 
hold on; 
plot(corners_close.Location(:, 1), corners_close.Location(:, 2), 'r+', 'MarkerSize', 5);
title('Close FOV');

% Medium FOV Image Plot
subplot(1, 3, 2); 
imshow(img_medium); 
hold on; 
plot(corners_medium.Location(:, 1), corners_medium.Location(:, 2), 'r+', 'MarkerSize', 5);
title('Medium FOV');

% Far POV Image Plot
subplot(1, 3, 3); 
imshow(img_far); 
hold on; 
plot(corners_far.Location(:, 1), corners_far.Location(:, 2), 'r+', 'MarkerSize', 5);
title('Far POV');

% Step 5: Select two patterns (corners) from one of the images
disp('Select the first corner feature');
[x1, y1] = ginput(1); % User selects first feature
pattern1_current = [x1, y1];

disp('Select the second corner feature');
[x2, y2] = ginput(1); % User selects second feature
pattern2_current = [x2, y2];

% Plot the selected features on the first image
figure; imshow(img_close); hold on;
plot(corners_close.Location(:, 1), corners_close.Location(:, 2), 'r+', 'MarkerSize', 5);
plot(x1, y1, 'ro', 'MarkerSize', 10, 'DisplayName', 'Feature 1 (Current)');
plot(x2, y2, 'go', 'MarkerSize', 10, 'DisplayName', 'Feature 2 (Current)');
legend;

% Step 6: Define desired positions for the selected features
disp('Click on the desired position for Feature 1');
[desired_x1, desired_y1] = ginput(1); % User selects desired position
pattern1_desired = [desired_x1, desired_y1];

disp('Click on the desired position for Feature 2');
[desired_x2, desired_y2] = ginput(1); % User selects desired position
pattern2_desired = [desired_x2, desired_y2];

% Control parameters
lambda = 0.1; % Control gain
tolerance = 2; % Stop when the error is within 2 pixels

% Step 7: Visual Servoing Control Loop
for iter = 1:100
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

    % Visualize the movement
    clf;
    imshow(img_close); hold on;
    plot(corners_close.Location(:, 1), corners_close.Location(:, 2), 'r+', 'MarkerSize', 5);
    plot(pattern1_current(1), pattern1_current(2), 'ro', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 1 (Current)');
    plot(pattern2_current(1), pattern2_current(2), 'go', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 2 (Current)');
    plot(pattern1_desired(1), pattern1_desired(2), 'rx', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 1 (Desired)');
    plot(pattern2_desired(1), pattern2_desired(2), 'gx', 'MarkerSize', 10, ...
        'DisplayName', 'Feature 2 (Desired)');
    legend;
    title(['Iteration: ', num2str(iter)]);
    pause(0.1); % Pause for visualization

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

disp('Visual servoing complete.');
