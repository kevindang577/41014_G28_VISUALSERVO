clc;
clear;
close all;

%% Step 1: Initialize the Intel RealSense Camera
pipe = realsense.pipeline();
cfg = realsense.config();
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
profile = pipe.start(cfg);
disp('Intel RealSense D435 initialized.');
pause(1); % Allow the camera to stabilize

%% Step 2: Capture Calibration Images
squareSize = 25; % Checkerboard square size in millimeters
numImages = 10; % Number of calibration images

% Use the system's temporary folder
calibrationFolder = fullfile(tempdir, 'calibration_images');

% Create the folder if it doesn't exist
if ~isfolder(calibrationFolder)
    mkdir(calibrationFolder);
end

disp(['Saving calibration images to: ', calibrationFolder]);
disp('Press any key to capture each calibration image...');

% Capture calibration images and save them
imageFiles = cell(1, numImages); % Store file paths
for i = 1:numImages
    disp(['Capturing image ', num2str(i)]);
    pause; % Wait for user input before capturing each image
    frames = pipe.wait_for_frames();
    colorFrame = frames.get_color_frame();
    img = permute(reshape(colorFrame.get_data(), [3, 640, 480]), [3, 2, 1]);
    
    % Save the image to the temporary folder
    fileName = fullfile(calibrationFolder, ['image_', num2str(i), '.png']);
    imwrite(img, fileName);
    imageFiles{i} = fileName;
    
    imshow(img), title(['Calibration Image ', num2str(i)]);
end

% Stop the camera temporarily after calibration
pipe.stop();

%% Step 3: Detect Checkerboard Points for Calibration
[imagePoints, boardSize] = detectCheckerboardPoints(imageFiles);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Estimate camera parameters
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    'ImageSize', size(imread(imageFiles{1}), [1, 2]));

disp('Camera calibration completed.');
disp(['Mean Reprojection Error: ', num2str(cameraParams.MeanReprojectionError)]);

% Save calibration parameters for future use
save('cameraParams.mat', 'cameraParams');

%% Step 4: Restart the Camera and Capture an Image for Detection
disp('Restarting camera for shape detection...');
pipe.start(cfg);
pause(1); % Allow the camera to stabilize

frames = pipe.wait_for_frames();
colorFrame = frames.get_color_frame();

if isempty(colorFrame)
    error('Failed to capture a frame from the RealSense D435.');
end

img = permute(reshape(colorFrame.get_data(), [3, 640, 480]), [3, 2, 1]);

% Undistort the image using the calibration parameters
undistortedImg = undistortImage(img, cameraParams);

figure, imshow(undistortedImg), title('Undistorted Image');

%% Step 5: Pre-process the Image (Grayscale, Binarize, Fill Holes)
grayImg = rgb2gray(undistortedImg);
BW = imbinarize(grayImg, 'adaptive');
BW = imfill(BW, 'holes'); % Fill holes inside shapes

figure, imshow(BW), title('Binary Image');

%% Step 6: Detect Boundaries and Classify Shapes
[B, L] = bwboundaries(BW, 'noholes');

figure, imshow(undistortedImg), title('Detected Shapes with Classification');
hold on;

for k = 1:length(B)
    boundary = B{k};

    % Filter small contours by area
    area = polyarea(boundary(:, 2), boundary(:, 1));
    if area < 100
        continue;
    end

    % Approximate the polygon using reducepoly
    approxShape = reducepoly(boundary, 0.02);
    numCorners = size(approxShape, 1);

    % Calculate aspect ratio
    minX = min(approxShape(:, 2));
    maxX = max(approxShape(:, 2));
    minY = min(approxShape(:, 1));
    maxY = max(approxShape(:, 1));
    aspectRatio = (maxX - minX) / (maxY - minY);

    % Classify the shape
    if numCorners == 3
        shapeType = 'Triangle';
    elseif numCorners == 4
        if aspectRatio >= 0.9 && aspectRatio <= 1.1
            shapeType = 'Square';
        else
            shapeType = 'Rectangle';
        end
    else
        shapeType = 'Other';
    end

    % Plot the shape boundary and label
    plot(approxShape(:, 2), approxShape(:, 1), 'g', 'LineWidth', 2);
    text(minX, minY, shapeType, 'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold');
end

hold off;

%% Step 7: Stop the RealSense Pipeline
pipe.stop();
disp('RealSense pipeline stopped.');
