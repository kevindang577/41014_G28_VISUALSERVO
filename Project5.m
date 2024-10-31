clc;
clear;
close all;

% Load and process the pattern image
patternImage = imread('1_Color.png'); % Load the pattern image
grayImage = rgb2gray(patternImage); % Convert to grayscale

% Apply Gaussian blurring to reduce noise and edge pixelation effects
grayImage = imgaussfilt(grayImage, 2); % Adjust the sigma value as needed

% Harris corner detection with adjusted parameters to reduce noise
corners = detectHarrisFeatures(grayImage, 'MinQuality', 0.1, 'FilterSize', 5); % Increase MinQuality to filter out weaker corners
cornerPoints = corners.Location; % Extract corner coordinates

% Filter nearby corner points to retain only unique/distinct corners
% Use pdist2 to find pairs of close points and remove duplicates
distThreshold = 10; % Distance threshold to filter close points
toKeep = true(size(cornerPoints, 1), 1); % Logical array to mark points to keep
for i = 1:size(cornerPoints, 1) - 1
    if toKeep(i)
        distances = pdist2(cornerPoints(i, :), cornerPoints(i+1:end, :));
        closePoints = find(distances < distThreshold);
        toKeep(i + closePoints) = false; % Mark close points as duplicates
    end
end
cornerPoints = cornerPoints(toKeep, :); % Filtered corner points

% Display the corners on the image for verification
figure;
imshow(grayImage);
hold on;
plot(cornerPoints(:,1), cornerPoints(:,2), 'r*');
title('Filtered Detected Corners');

% Convert corner points to the robot’s workspace coordinates
imageSize = size(grayImage);
workspaceX = linspace(-0.5, 0.5, imageSize(2)); % Adjust based on workspace size
workspaceY = linspace(0, 1, imageSize(1));
workspacePoints = [workspaceX(round(cornerPoints(:,1)))', workspaceY(round(cornerPoints(:,2)))', repmat(0.8, length(cornerPoints), 1)]; % Z=0.8

%% Set up the environment with table and UR3
figure;

% Load in the table
Table = PlaceObject('counter.ply');
Table_vertices = get(Table, 'Vertices');
transformedVerticesT = [0.5 * Table_vertices, ones(size(Table_vertices, 1), 1)] * troty(-pi/2)' * transl(0, 0, 0)';
set(Table, 'Vertices', transformedVerticesT(:, 1:3));

% Load the UR3 robot
robot = UR3(transl(0, 0, 0.75));

% Set up the 3D plot for the robot and environment
subplot(2,2,2); % 3D environment
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([0, 2]);
axis equal;
grid on;
view(3);
hold on;

% Plot the wall
surf([-1.5, 1.5; -1.5, 1.5], [1.5, 1.5; 1.5, 1.5], [0, 0; 2, 2], 'CData', imread('TriangleTrace2.png'), 'FaceColor', 'texturemap');

% Initialize trace for visualization
traceHandle = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5);

% Set up the 2D plot for end-effector path on X-Y plane
subplot(2,2,[3,4]); % Bottom row for the 2D end-effector path
endEffectorPathHandle = plot(NaN, NaN, 'b', 'LineWidth', 1.5); % Blue line for end-effector path
title('End-Effector Path (X-Y Plane)');
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis equal;
grid on;
hold on;

% Define end-effector orientation for tracing (pointing down)
targetOri = trotx(-pi/2) * troty(pi/2);

% Move UR3 to each detected corner
tracePoints = []; % Store the path for continuous line update
xyPathPoints = []; % Store end-effector (x, y) coordinates for the 2D plot

for i = 1:size(workspacePoints, 1)
    targetPoint = workspacePoints(i, :);
    moveUR3ToPoint(robot, targetPoint, targetOri);
    
    % Append the current target position to the trace
    tracePoints = [tracePoints; targetPoint];
    xyPathPoints = [xyPathPoints; targetPoint(1), targetPoint(2)]; % Only X-Y for 2D plot
    
    % Update the 3D trace on the plot
    set(traceHandle, 'XData', tracePoints(:,1), 'YData', tracePoints(:,2), 'ZData', tracePoints(:,3));
    
    % Update the 2D end-effector path on the X-Y plane
    set(endEffectorPathHandle, 'XData', xyPathPoints(:,1), 'YData', xyPathPoints(:,2));
    
    pause(0.05); % Adjust for speed of tracing
end

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
    Q_destination = SE3(transl(targetPos) * targetOri);
    initialGuess = robot.model.getpos();

    % Set options for inverse kinematics with relaxed constraints
    ikine_options = {'tol', 1e-4, 'lambda', 0.5, 'lambdamin', 0.01, 'ilimit', 1000, 'forceSoln', true, 'mask', [1 1 1 0 0 0]};
    
    Q = robot.model.ikine(Q_destination.T, 'q0', initialGuess, ikine_options{:});
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
