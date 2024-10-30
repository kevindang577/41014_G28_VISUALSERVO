clc;
close all;
clear;

%% Function for loading in Environment + UR3

% Square Pattern
% patternImage = imread('SquarePatternTrace.png'); % Load the pattern image
% Triangle Pattern
patternImage = imread('TriangleTrace.png'); % Load the pattern image
% Composite Shape
% patternImage = imread('squarepattern1.png'); % Load the pattern image


% Display the wall with the pattern texture in the workspace
figure;
hold on;
wall = surf([-1.5,1.5;-1.5,1.5],[1.5,1.5;1.5,1.5],[0,0;2,2],'CData',patternImage,'FaceColor','texturemap');

% Harris corner detection on the pattern
grayImage = rgb2gray(patternImage); % Convert to grayscale
corners = detectHarrisFeatures(grayImage); % Detect Harris corners
cornerPoints = corners.selectStrongest(20).Location; % Select the 20 strongest corners

% Load the table
Table = PlaceObject('counter.ply');
Table_vertices = get(Table, 'Vertices');
transformedVerticesT = [0.5 * Table_vertices, ones(size(Table_vertices,1),1)] * troty(-pi/2)' * transl(0,0,0)';
set(Table, 'Vertices', transformedVerticesT(:, 1:3));

% Load UR3
robot = UR3(transl(0,0,0.75));

% Set axis limits
xlim([-1.5, 1.5]);
ylim([-1.5, 1.5]);
zlim([0, 2]);

axis equal;
grid on;
view(3);

%% Map Corners to Wall in Workspace Coordinates

% Define the scaling factors and offsets to map image to workspace
imageSize = size(grayImage);
workspaceX = linspace(-1.5, 1.5, imageSize(2)); % Scale x-coordinates
workspaceY = linspace(0, 2, imageSize(1)); % Scale z-coordinates
wallPointsWorkspace = [workspaceX(round(cornerPoints(:,1)))', repmat(1.5, size(cornerPoints,1), 1), workspaceY(round(cornerPoints(:,2)))']; % Map to wall in workspace

%% Move UR3 to Follow Detected Corner Points

% Define end-effector orientation (pointing at the wall)
targetOri = trotx(-pi/2) * troty(pi/2) * trotz(0);

% Loop over each detected corner point
for i = 1:size(wallPointsWorkspace, 1)
    targetPoint = wallPointsWorkspace(i, :);
    moveUR3ToPoint(robot, targetPoint, targetOri);
    pause(0.5); % Pause to visualize movement
end

%% Local Functions

function moveUR3ToPoint(robot, targetPoint, targetOri)
    % moveUR3ToPoint - Moves the UR3 robot to a specific point in workspace
    % Inputs:
    %   robot - The UR3 robot object
    %   targetPoint - A 3-element vector [x, y, z] representing the target coordinates
    %   targetOri - Orientation matrix for the end-effector
    
    % Generate the joint configurations to reach the target using FindqMatrix logic
    [matrixSignal, qMatrix] = FindqMatrix(robot, targetPoint, targetOri);
    if matrixSignal
        % Run the movement using a controlled animation
        run(robot, qMatrix);
    else
        disp('Unable to find a valid path to the target position.');
    end
end

function [result, qMatrix] = FindqMatrix(robot, targetPos, targetOri)
    % This function calculates the joint configurations to reach a target
    qMatrix = [];
    cur_object = targetPos;
    cur_pos = robot.model.fkine(robot.model.getpos).T;
    
    % Ensure `cur_pos` is a valid transformation
    if (cur_pos(3,4) - 1.5) < 0.2
        cur_pos(3,4) = cur_pos(3,4) + 0.2;
    end
    
    % Create the destination transformations
    Q_destination{2} = SE3(cur_pos);  % Current pose, adjusted height if needed
    Q_destination{3} = SE3(transl(cur_object + [0, 0, 0.3])) * SE3(targetOri); % Move above target
    Q_destination{4} = SE3(transl(cur_object)) * SE3(targetOri); % Approach target directly
    
    % Initial joint configuration guesses
    Q{1} = robot.model.getpos();
    Q{2} = [];
    Q{3} = [];
    Q{4} = [];
    
    % Use previous configuration as an initial guess for subsequent steps
    initialGuess = Q{1};
    
    % Define ikine options with force solution
    ikine_options = {
        'tol', 1e-4, ...
        'lambda', 0.5, ...
        'lambdamin', 0.01, ...
        'ilimit', 1000, ...
        'forceSoln', true, ...
        'mask', [1 1 1 0 0 0]
    };
    
    for pt = 2:4
        count = 1;
        
        while count <= 1000  % Limit iterations for stability
            % Attempt to calculate the inverse kinematics with adjusted options
            Q{pt} = robot.model.ikine(Q_destination{pt}.T, 'q0', initialGuess, ikine_options{:});
            
            if ~isempty(Q{pt})
                result = 1;
                % Generate smooth trajectory between configurations
                qMatrix = [qMatrix; jtraj(Q{pt-1}, Q{pt}, 100)];
                initialGuess = Q{pt}; % Update the initial guess for the next iteration
                break;
            else
                result = 0;
                count = count + 1;
            end
        end
        if ~result
            disp('Failed to generate a path to target.');
            break;
        end
    end
end

function run(robot, qMatrix)
    % Main function to run the robot through the qMatrix and plot a trace
    % Initialize trace points and plot handle
    tracePoints = []; % Array to store end-effector positions
    traceHandle = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5); % Initialize the line object
    
    if ~isempty(qMatrix)
        for i = 1:size(qMatrix, 1)
            % Animate the robot
            robot.model.animate(qMatrix(i,:));
            
            % Get the current end-effector position
            endEffectorPose = robot.model.fkine(qMatrix(i,:));
            
            if ~isempty(endEffectorPose) && size(endEffectorPose,1) == 4
                % Extract the (x, y, z) position of the end-effector
                endEffectorPosition = endEffectorPose(1:3, 4)';
                
                % Append the new position to tracePoints
                tracePoints = [tracePoints; endEffectorPosition];
                
                % Update the line object data to reflect the trace
                set(traceHandle, 'XData', tracePoints(:,1), 'YData', tracePoints(:,2), 'ZData', tracePoints(:,3));
            end
            
            drawnow;
        end
    else
        disp('qMatrix is empty, no movement to execute.');
    end
end



