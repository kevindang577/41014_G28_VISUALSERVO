% Clear workspace and initialize
clc;
clear;
close all;

% Load the UR3 robot model from Peter Corke's Robotics Toolbox
%% ur3 = loadrobot('universalUR3', 'DataFormat', 'column', 'Gravity', [0, 0, -9.81]);

ur3 = UR3;

% Define camera parameters for simulation
focal_length = 800; 
principal_point = [320, 240]; % Image center
resolution = [640, 480]; % Image resolution

% Create a monocular pinhole camera model in the Robotics Toolbox
cam = CentralCamera('focal', focal_length, ...
                    'pixel', 10e-6, ...
                    'resolution', resolution, ...
                    'centre', principal_point, ...
                    'name', 'Simulation Camera');

% Set initial camera pose (position and orientation in SE(3))
Tc0 = transl(0.5, 0, 0.5) * trotx(pi); % Positioned above and looking down

% Visualize the camera and UR3 model in 3D space
figure;
hold on;
cam.plot_camera('Tcam', Tc0); % Plot the camera in its initial pose
view(3); axis equal; grid on;
title('UR3 Robot with Simulated Camera');
show(ur3, homeConfiguration(ur3));

% Simulate two feature points in the world (in 3D space)
P1 = [0.2; 0.2; 0];  % 3D coordinates of feature 1 in world frame
P2 = [0.2; -0.2; 0]; % 3D coordinates of feature 2 in world frame

% Project the 3D points into the camera's 2D image plane
p1 = cam.plot(P1, 'Tcam', Tc0, 'label', 'P1'); % Initial 2D projection of P1
p2 = cam.plot(P2, 'Tcam', Tc0, 'label', 'P2'); % Initial 2D projection of P2

% Define desired 2D positions in the image plane (target positions)
p_star1 = [320, 240]; % Desired pixel location for pattern 1
p_star2 = [400, 300]; % Desired pixel location for pattern 2

% Define control gain
lambda = 0.01; 

% Set simulation parameters
max_iterations = 100; % Maximum iterations for control loop
threshold = 5; % Error threshold for convergence

% Initialize the camera pose for the control loop
Tc = Tc0;

for k = 1:max_iterations
    % Project the 3D points into the current image plane
    p1 = cam.project(P1, 'Tcam', Tc); 
    p2 = cam.project(P2, 'Tcam', Tc);
    
    % Compute the 2D error between current and desired positions
    error1 = p_star1 - p1;
    error2 = p_star2 - p2;
    error = [error1'; error2']; % Combine errors into one vector

    % Display current error
    disp(['Iteration ', num2str(k), ': Error = ', num2str(norm(error))]);

    % Check if the error is below the threshold
    if norm(error) < threshold
        disp('Target reached!');
        break;
    end

    % Compute the interaction matrix for the two points
    L1 = cam.visjac_p(p1, 1); % Interaction matrix for pattern 1
    L2 = cam.visjac_p(p2, 1); % Interaction matrix for pattern 2

    % Combine the interaction matrices
    L = [L1; L2]; 

    % Compute the control velocity (camera twist)
    v = lambda * pinv(L) * error;

    % Update the camera pose using the computed velocity
    Tc = Tc * se3(v * 0.1); % Apply velocity with a small time step

    % Update visualization
    cam.plot_camera('Tcam', Tc); 
    drawnow;
end

disp('Simulation completed.');

% Initialize the robot's joint configuration
q = homeConfiguration(ur3); % Start from the home position

% Set the IK solver for the UR3
ik = inverseKinematics('RigidBodyTree', ur3);
weights = [0, 0, 0, 1, 1, 1]; % Weights for IK solution
endEffector = 'tool0'; % UR3 end-effector

for k = 1:max_iterations
    % Project the 3D points into the current image plane
    p1 = cam.project(P1, 'Tcam', Tc); 
    p2 = cam.project(P2, 'Tcam', Tc);

    % Compute the 2D error between current and desired positions
    error1 = p_star1 - p1;
    error2 = p_star2 - p2;
    error = [error1'; error2'];

    % Check if the error is below the threshold
    if norm(error) < threshold
        disp('Target reached!');
        break;
    end

    % Compute the interaction matrix and control velocity
    L1 = cam.visjac_p(p1, 1);
    L2 = cam.visjac_p(p2, 1);
    L = [L1; L2];
    v = lambda * pinv(L) * error;

    % Compute the robot's Jacobian and joint velocities
    J = geometricJacobian(ur3, q, endEffector);
    q_dot = pinv(J) * v;

    % Update the robot's joint configuration
    dt = 0.1; % Time step
    q = q + q_dot * dt;

    % Visualize the updated robot configuration
    show(ur3, q, 'PreservePlot', false);
    cam.plot_camera('Tcam', Tc); 
    drawnow;
end

disp('Robot control simulation completed.');
