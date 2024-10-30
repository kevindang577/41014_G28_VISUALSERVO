clc;
clear;
close all;

%% Load and process the pattern image
patternImage = imread('CompoundShape.png'); % Load the pattern image (use your specific image here)
grayImage = rgb2gray(patternImage); % Convert to grayscale

% Apply Gaussian blur to reduce noise
blurredImage = imgaussfilt(grayImage, 2); % Adjust sigma for smoothness

% Harris corner detection to find key points
corners = detectHarrisFeatures(blurredImage);
cornerPoints = corners.Location; % Extract corner coordinates

% Order the corner points for tracing
centroid = mean(cornerPoints, 1);
angles = atan2(cornerPoints(:,2) - centroid(2), cornerPoints(:,1) - centroid(1));
[~, sortIdx] = sort(angles);
orderedCornerPoints = cornerPoints(sortIdx, :);

% Display the ordered points on the image for verification
figure;
imshow(blurredImage);
hold on;
plot(orderedCornerPoints(:,1), orderedCornerPoints(:,2), 'r*-');
title('Ordered Detected Corners');

% Convert ordered corner points to workspace coordinates
imageSize = size(grayImage);
workspaceX = linspace(-0.5, 0.5, imageSize(2)); % Adjust based on workspace size
workspaceY = 1; % Keep Y constant in workspace
workspaceZ = linspace(0.2, 0.8, imageSize(1)); % Adjust based on workspace height
wallY = 1; % This is the Y position on which the wall pattern is located

workspacePoints = [workspaceX(round(orderedCornerPoints(:,1)))', ...
                   repmat(wallY, length(orderedCornerPoints), 1), ...
                   workspaceZ(round(imageSize(1) - orderedCornerPoints(:,2) + 1))'];
workspacePoints(:,3) = workspacePoints(:,3) + 0.5; % Adjust the offset as needed


% Append the first point to the end to close the shape
workspacePoints = [workspacePoints; workspacePoints(1, :)];

%% Set up the environment with table and UR3
figure; % Open a new figure for the 3D environment
hold on;
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([0, 2]);
axis equal;
grid on;
view(3);

% Load in the table
Table = PlaceObject('counter.ply');
Table_vertices = get(Table, 'Vertices');
transformedVerticesT = [0.5 * Table_vertices, ones(size(Table_vertices, 1), 1)] * troty(-pi/2)' * transl(0, 0, 0)';
set(Table, 'Vertices', transformedVerticesT(:, 1:3));

% Load the UR3 robot
robot = UR3(transl(0, 0, 0.75));

% Initialize trace for visualization in 3D plot
traceHandle = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5);

% Set up 2D figure for end-effector path
figure; % New figure for the 2D plot
endEffectorPathHandle = plot(NaN, NaN, 'b', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Z Position (m)');
title('End-Effector Path (X-Z Plane)');
grid on;
hold on;

% Define end-effector orientation for tracing (pointing down)
targetOri = trotx(-pi/2) * troty(pi/2);

% Move UR3 to each detected corner, maintaining the constant Y position
tracePoints = []; % Store the path for continuous line update in 3D
xzPathPoints = []; % Store end-effector (x, z) coordinates for the 2D plot

for i = 1:size(workspacePoints, 1)
    targetPoint = workspacePoints(i, :);
    
    % Override the Y-coordinate to maintain a fixed position
    targetPoint(2) = wallY;
    
    moveUR3ToPoint(robot, targetPoint, targetOri);
    
    % Append the current target position to the trace
    tracePoints = [tracePoints; targetPoint];
    xzPathPoints = [xzPathPoints; targetPoint(1), targetPoint(3)]; % Only X-Z for 2D plot
    
    % Update the 3D trace on the plot
    set(traceHandle, 'XData', tracePoints(:,1), 'YData', tracePoints(:,2), 'ZData', tracePoints(:,3));
    
    % Update the 2D end-effector path on the X-Z plane
    set(endEffectorPathHandle, 'XData', xzPathPoints(:,1), 'YData', xzPathPoints(:,2));
    
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
