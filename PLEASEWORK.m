% Part 2: UR3e Simulation with Intel RealSense D455 and Visual Servoing
% Requirements: MATLAB 2024a, Peter Corke's Robotics Toolbox, Computer Vision Toolbox

% Load the UR3e robot model from Peter Corke's Robotics Toolbox
ur3e = loadrobot('universalUR3e', 'DataFormat', 'row');

% Define the initial configuration for the UR3e (home position)
q_initial = homeConfiguration(ur3e);

% Set up the inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', ur3e);
ikWeights = [0.25 0.25 0.25 1 1 1];  % Weights for the IK solution
ikInitGuess = q_initial;  % Use initial configuration as a starting guess

% Set up the figure for displaying the simulation and camera output
figure;
ax = show(ur3e, q_initial, 'PreservePlot', false);
hold on;
title('UR3e Robot with Intel RealSense D455 at End-Effector');
axis([-1 1 -1 1 0 2]);  % Set axis limits

% Initialize the Intel RealSense D455 pipeline
pipeline = realsense.pipeline();
config = realsense.config();
config.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
pipeline.start(config);

disp('Starting UR3e simulation with Intel RealSense D455...');
prevCorner = [];  % Store the previous corner position for direction calculation

try
    % Real-time Loop: Capture frames and move UR3e end-effector
    while true
        % Capture the next frame from the Intel D455
        frameset = pipeline.wait_for_frames();
        color_frame = frameset.get_color_frame();
        if isempty(color_frame)
            disp('Invalid frame. Skipping...');
            continue;
        end

        % Convert the frame to MATLAB format
        frame = permute(reshape(color_frame.get_data(), [3, 640, 480]), [3, 2, 1]);
        grayFrame = rgb2gray(frame);  % Convert to grayscale

        % Perform Harris corner detection
        corners = detectHarrisFeatures(grayFrame);

        % Display the live camera feed in a subplot
        subplot(1, 2, 1);
        imshow(frame);
        hold on;
        plot(corners.selectStrongest(50));  % Display detected corners
        hold off;
        title('Intel RealSense D455 Live Feed');

        % Calculate movement direction based on corner positions
        if ~isempty(corners.Location) && ~isempty(prevCorner)
            % Find the nearest detected corner to the previous one
            distances = vecnorm(corners.Location - prevCorner, 2, 2);
            [~, nearestIdx] = min(distances);
            nearestCorner = corners.Location(nearestIdx, :);

            % Calculate the displacement vector (X, Y)
            dx = nearestCorner(1) - prevCorner(1);
            dy = nearestCorner(2) - prevCorner(2);

            % Update the end-effector position based on the displacement
            ee_pose = getTransform(ur3e, ikInitGuess, 'tool0');  % Get current EE pose
            ee_pose(1, 4) = ee_pose(1, 4) + dx * 0.001;  % Apply small X movement
            ee_pose(2, 4) = ee_pose(2, 4) + dy * 0.001;  % Apply small Y movement

            % Solve inverse kinematics for the new end-effector pose
            [q_new, ~] = ik('tool0', ee_pose, ikWeights, ikInitGuess);

            % Update the robot's configuration
            show(ur3e, q_new, 'PreservePlot', false, 'Parent', ax);
            ikInitGuess = q_new;  % Update initial guess for next iteration

            % Print movement directions to the console
            if dx > 0
                disp('Move right (X+)');
            elseif dx < 0
                disp('Move left (X-)');
            end
            if dy > 0
                disp('Move down (Y-)');
            elseif dy < 0
                disp('Move up (Y+)');
            end

            % Update the previous corner to the current nearest corner
            prevCorner = nearestCorner;
        elseif ~isempty(corners.Location)
            % Initialize the previous corner if this is the first frame
            prevCorner = corners.Location(1, :);
        end

        % Display the updated robot position in the second subplot
        subplot(1, 2, 2);
        show(ur3e, q_new, 'PreservePlot', false);
        title('UR3e Robot with Updated End-Effector Position');

        % Exit if 'q' is pressed
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
