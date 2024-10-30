% Part 1: Visual Servoing with Intel RealSense D455, Harris Corner Detection, and Edge Detection
% Requirements: MATLAB 2024a, Computer Vision Toolbox, Intel RealSense SDK for MATLAB

% Initialize RealSense pipeline
pipeline = realsense.pipeline();
config = realsense.config();

% Configure the pipeline to enable the color stream from D455
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);

% Start streaming from the camera
pipeline.start(config);
disp('Starting Intel RealSense D455 stream with Edge and Corner Detection...');

% Initialize previous corner location (empty initially)
prevCorner = [];

try
    % Real-time Video Capture Loop
    while true
        % Wait for the next set of frames
        frameset = pipeline.wait_for_frames();

        % Extract the color frame from the frameset
        color_frame = frameset.get_color_frame();

        % Validate the frame by checking if data exists
        frame_data = color_frame.get_data();
        if isempty(frame_data)
            disp('Invalid frame. Skipping...');
            continue;
        end

        % Convert the RealSense frame to MATLAB image format
        frame = permute(reshape(frame_data, [3, 640, 480]), [3, 2, 1]);

        % Convert to grayscale for contour detection and edge detection
        grayFrame = rgb2gray(frame);

        % Perform edge detection using Canny method
        edges = edge(grayFrame, 'Canny');

        % Detect straight lines using the Hough Transform
        [H, T, R] = hough(edges);
        P = houghpeaks(H, 50, 'Threshold', 0.3 * max(H(:)));  % Find peaks in Hough space
        lines = houghlines(edges, T, R, P, 'FillGap', 10, 'MinLength', 50);  % Find lines

        % Perform Harris corner detection on the whole frame
        corners = detectHarrisFeatures(grayFrame);

        % Prepare figure with edges and corners overlaid
        imshow(frame);
        hold on;

        % Plot the detected lines if they correspond to corners
        for k = 1:length(lines)
            line = lines(k);
            % Extract the start and end points of the line
            x1 = line.point1(1); y1 = line.point1(2);
            x2 = line.point2(1); y2 = line.point2(2);

            % Check if the line endpoints match or are near detected corners
            dist1 = vecnorm(corners.Location - [x1, y1], 2, 2);
            dist2 = vecnorm(corners.Location - [x2, y2], 2, 2);

            % Threshold for matching corners (within 5 pixels)
            if min(dist1) < 5 && min(dist2) < 5
                % Plot the line in green if both endpoints align with corners
                plot([x1, x2], [y1, y2], 'g-', 'LineWidth', 2);

                % Mark the corners with red circles
                plot(x1, y1, 'ro', 'MarkerSize', 5);
                plot(x2, y2, 'ro', 'MarkerSize', 5);
            end
        end

        % Compute and print movement directions to the next nearest corner
        if ~isempty(corners.Location) && ~isempty(prevCorner)
            % Calculate distances to all detected corners from the previous corner
            distances = vecnorm(corners.Location - prevCorner, 2, 2);
            [~, nearestIdx] = min(distances);
            nearestCorner = corners.Location(nearestIdx, :);

            % Compute displacement vector (X, Y)
            displacement = nearestCorner - prevCorner;

            % Output Cartesian directions for movement
            if displacement(1) > 0
                disp('Move right (X+)');
            elseif displacement(1) < 0
                disp('Move left (X-)');
            end

            if displacement(2) > 0
                disp('Move down (Y-)');
            elseif displacement(2) < 0
                disp('Move up (Y+)');
            end

            % Optional: Output Z-direction (based on size change)
            if norm(displacement) > 50  % Example threshold for Z-direction
                disp('Move backward (Z-)');
            else
                disp('Move forward (Z+)');
            end

            % Update the previous corner to the current nearest corner
            prevCorner = nearestCorner;
        elseif ~isempty(corners.Location)
            % Initialize prevCorner if this is the first frame with corners
            prevCorner = corners.Location(1, :);
        end

        hold off;

        % Exit loop if 'q' is pressed
        if ~isempty(get(gcf, 'CurrentCharacter')) && get(gcf, 'CurrentCharacter') == 'q'
            disp('Exiting...');
            break;
        end

        % Control frame rate
        pause(0.1);
    end
catch ME
    disp(['Error: ', ME.message]);
end

% Stop the pipeline and release resources
pipeline.stop();
disp('Camera stream stopped.');
