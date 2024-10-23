% Install Robotics Toolbox if needed (uncomment to install)
% matlab.addons.install('rtb.zip'); % Use appropriate path to the toolbox

% Load the UR3 robot model
ur3 = loadrobot('universalUR3', 'DataFormat', 'column');

% Initialize camera parameters
focal_length = 800; % in pixels (example value)
principal_point = [320, 240]; % Assuming 640x480 resolution

% Create a camera model (monocular pinhole camera)
cam = CentralCamera('focal', focal_length, ...
                    'pixel', 10e-6, ...
                    'resolution', [640, 480], ...
                    'centre', principal_point);

% Load the patterns (e.g., QR codes or AprilTags)
pattern1 = imread('pattern1.png');
pattern2 = imread('pattern2.png');

% Initialize feature detector and matcher
featureDetector = detectSURFFeatures(pattern1);
[features1, validPoints1] = extractFeatures(pattern1, featureDetector);

% Start live camera feed (for manual use first)
cameraObj = webcam(); % Ensure your camera is connected

% Capture an image and detect features from live feed
frame = snapshot(cameraObj);
grayFrame = rgb2gray(frame);
featureDetector = detectSURFFeatures(grayFrame);
[featuresFrame, validPointsFrame] = extractFeatures(grayFrame, featureDetector);

% Match the features between the live frame and pattern
indexPairs = matchFeatures(features1, featuresFrame);
matchedPoints1 = validPoints1(indexPairs(:, 1));
matchedPointsFrame = validPointsFrame(indexPairs(:, 2));

% Display matched features (for visualization)
figure;
showMatchedFeatures(pattern1, grayFrame, matchedPoints1, matchedPointsFrame, 'montage');

% Define two desired 2D image points (e.g., from the two patterns)
p_star1 = [320, 240]; % Desired position for pattern 1
p_star2 = [400, 300]; % Desired position for pattern 2

% Real-time tracking loop
while true
    frame = snapshot(cameraObj); % Capture current frame
    grayFrame = rgb2gray(frame);
    [featuresFrame, validPointsFrame] = extractFeatures(grayFrame, featureDetector);
    
    % Match features
    indexPairs = matchFeatures(features1, featuresFrame);
    matchedPointsFrame = validPointsFrame(indexPairs(:, 2));
    
    % Get the current 2D positions
    p1 = matchedPointsFrame.Location(1, :); % Current position of pattern 1
    p2 = matchedPointsFrame.Location(2, :); % Current position of pattern 2
    
    % Compute error (difference between desired and current positions)
    error1 = p_star1 - p1;
    error2 = p_star2 - p2;
    
    % Interaction matrix (Jacobian-like matrix for image-based visual servoing)
    L = [-1, 0, p1(1); 
          0, -1, p1(2)]; 

    % Control law: v = lambda * pinv(L) * error
    lambda = 0.01; % Gain factor
    v = lambda * pinv(L) * [error1'; error2']; % Control velocity

    % Display real-time error for visualization
    disp(['Error1: ', num2str(error1), '  Error2: ', num2str(error2)]);

    % (Optional) Break the loop if error is small enough
    if norm(error1) < 5 && norm(error2) < 5
        disp('Target reached!');
        break;
    end
end

% Define the initial joint configuration of the UR3
q_init = homeConfiguration(ur3);

% Set up the robot's IK solver
ik = inverseKinematics('RigidBodyTree', ur3);
weights = [0 0 0 1 1 1]; % Weights for the IK solver
endEffector = 'tool0'; % Name of the UR3 end-effector

% Real-time control loop for the UR3 with visual servoing
q = q_init; % Start with the initial configuration

while true
    % Capture current frame and compute error (as in previous step)
    frame = snapshot(cameraObj);
    grayFrame = rgb2gray(frame);
    [featuresFrame, validPointsFrame] = extractFeatures(grayFrame, featureDetector);
    indexPairs = matchFeatures(features1, featuresFrame);
    matchedPointsFrame = validPointsFrame(indexPairs(:, 2));
    p1 = matchedPointsFrame.Location(1, :);
    error1 = p_star1 - p1;

    % Compute camera velocity using interaction matrix
    v = lambda * pinv(L) * error1'; 

    % Convert camera velocity to joint velocity using Jacobian
    J = geometricJacobian(ur3, q, endEffector);
    q_dot = pinv(J) * v;

    % Update joint angles using Euler integration
    dt = 0.1; % Time step
    q = q + q_dot * dt;

    % Apply new joint angles to the UR3 model
    show(ur3, q, 'PreservePlot', false);
    drawnow;

    % Check if the target is reached
    if norm(error1) < 5
        disp('Target reached by UR3!');
        break;
    end
end
