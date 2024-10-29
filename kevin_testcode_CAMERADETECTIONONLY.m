clc;
clear;
close all;

%% Step 1: Initialize the Intel RealSense Pipeline
pipe = realsense.pipeline();
cfg = realsense.config();
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);

profile = pipe.start(cfg);
disp('Intel RealSense D435 initialized.');
pause(1); % Allow time for the camera to stabilize

%% Step 2: Capture Image from D435 Camera
frames = pipe.wait_for_frames();
colorFrame = frames.get_color_frame();

if isempty(colorFrame)
    error('Failed to capture a frame from the RealSense D435.');
end

img = permute(reshape(colorFrame.get_data(), [3, 640, 480]), [3, 2, 1]);

figure, imshow(img), title('Captured Image from Intel RealSense D435');

%% Step 3: Pre-process the Image (Convert to Grayscale and Binarize)
grayImg = rgb2gray(img);  
BW = imbinarize(grayImg, 'adaptive'); % Use adaptive thresholding for better results
BW = imfill(BW, 'holes');  % Fill holes inside the shapes

figure, imshow(BW), title('Binary Image');

%% Step 4: Detect Boundaries and Approximate Contours
[B, L] = bwboundaries(BW, 'noholes');

% Display the original image with contours and classifications
figure, imshow(img), title('Detected Shapes with Classification');
hold on;

for k = 1:length(B)
    boundary = B{k};  % Get boundary points

    % Filter by area to remove noise
    area = polyarea(boundary(:, 2), boundary(:, 1));
    if area < 100  % Ignore small objects
        continue;
    end

    % Use reducepoly to approximate the polygon
    approxShape = reducepoly(boundary, 0.02);
    numCorners = size(approxShape, 1);

    % Calculate the aspect ratio (Width / Height)
    minX = min(approxShape(:, 2));
    maxX = max(approxShape(:, 2));
    minY = min(approxShape(:, 1));
    maxY = max(approxShape(:, 1));
    aspectRatio = (maxX - minX) / (maxY - minY);

    % Classify shapes based on the number of corners
    if numCorners == 3
        shapeType = 'Triangle';
    elseif numCorners == 4
        % Use aspect ratio to differentiate between square and rectangle
        if aspectRatio >= 0.9 && aspectRatio <= 1.1
            shapeType = 'Square';
        else
            shapeType = 'Rectangle';
        end
    else
        shapeType = 'Other';
    end

    % Plot the shape boundary
    plot(approxShape(:, 2), approxShape(:, 1), 'g', 'LineWidth', 2);

    % Display the shape type
    text(minX, minY, shapeType, 'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold');
end

hold off;

%% Step 5: Stop the RealSense Pipeline
pipe.stop();
disp('RealSense pipeline stopped.');
