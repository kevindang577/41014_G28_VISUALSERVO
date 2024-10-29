clc;
close all;

%% Function for loading in Environment + UR3

% Load in table
Table = PlaceObject('counter.ply');
Table_vertices = get(Table,'Vertices');
transformedVerticesT = [0.5*Table_vertices,ones(size(Table_vertices,1),1)]*troty(-pi/2)'*transl(0,0,0)';
set(Table,'Vertices',transformedVerticesT(:,1:3));

% Loading UR3
robot = UR3(transl(0,0,0.75));

% Set limits for axis
xlim([-1.5, 1.5]) 
ylim([-1.5, 1.5]) 
zlim([0, 2])  

axis equal
grid on
view(3)

hold on

% Plot the wall
surf([-1.5,1.5;-1.5,1.5],[1.5,1.5;1.5,1.5],[0,0;2,2],'CData',imread('pattern1.png'),'FaceColor','texturemap');

% Define the target point
targetPoint = [0.5, 0.2, 0.8];

% Call the function to move the UR3
moveUR3ToPoint(robot, targetPoint);

%% Local Functions

function moveUR3ToPoint(robot, targetPoint)
    % moveUR3ToPoint - Moves the UR3 robot so that its end-effector points towards the target point.
    %
    % Inputs:
    %   robot - The UR3 robot object
    %   targetPoint - A 3-element vector [x, y, z] representing the target coordinates
    
    % Define the target orientation
    targetOri = trotx(-pi/2) * troty(0) * trotz(-pi/2); % Adjust orientation if needed

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
    
    for pt = 2:4
        count = 1;
        initialGuess = Q{pt-1}; % Use the previous joint configuration as the initial guess
        
        while count <= 1000  % Limit iterations for stability
            % Attempt to calculate the inverse kinematics
            Q{pt} = robot.model.ikine(Q_destination{pt}.T, 'q0', initialGuess, 'mask', [1 1 1 0 0 0], 'forceSln');
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
    % Main function to run the robot through the qMatrix
    if ~isempty(qMatrix)
        for i = 1:size(qMatrix, 1)
            % Animate the robot
            robot.model.animate(qMatrix(i,:));
            drawnow;
        end
    end
end
