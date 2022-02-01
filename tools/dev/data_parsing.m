source "./tools/utilities/geometry_helpers_2d.m"

% Function for computing the ground truth robot poses matrix
% INPUT:
% poses_ground_truth: is the vector which containts [x y theta]
% num_poses: is the number of poses in poses_ground_truth
%
% OUTPUT
% XR_true: is the robot poses as Matrices belonging to SE(2)
function XR_true = poses_ground_truth2SE(poses_ground_truth, num_poses)
    
    XR_true = zeros(3,3, num_poses); %initializing the matrix

    for i = 1:num_poses
        theta = poses_ground_truth(i).theta;

        %Rotation matrix part of XR_true
        XR_true(1,1,i) = cos(theta);
        XR_true(1,2,i) = -sin(theta);
        XR_true(2,1,i) = sin(theta);
        XR_true(2,2,i) = cos(theta);
        %Offset Vector
        XR_true(1,3, i) = poses_ground_truth(i).x;
        XR_true(2,3, i) = poses_ground_truth(i).y;
        XR_true(3,3, i) = 1; %Homogeneous transformations have as last row [0 0 1] 
    endfor
endfunction

%Function for generating a vector of ground truth landmarks positions
% INPUT:
% landmarks_ground_truth: is the vector containing coordinates [x y] of the landmark
% num_landmarks: is the number of positions of landmarks available
%
% OUTPUT:
% XL_true: is the vector of the positions of the landamarks assembled
function XL_true = positionLandmark_ground_truth(landmarks_ground_truth, num_landmarks)

    %Initializing the final vector of landmark positions
    XL_true = zeros(2, num_landmarks);
    for i=1:num_landmarks
        curr_landmark = landmarks_ground_truth(i);
        x = curr_landmark.x_pose;
        y = curr_landmark.y_pose;

        XL_true(1, i)  = x;
        XL_true(2, i) = y;
    endfor
endfunction

%Function for computing the initial guess of the robot pose matrix
%It is done by using the odometry measuerements
% INPUT:
% poses: is the vector which containts [x y theta]
% num_poses: is the number of poses in poses
% transitions: vector containing the offset between pose i-1 and i, composed by [t_x, t_y,t_theta]
% num_transitions: is the number of the transitions
%
% OUTPUT
% XR_guess: is the robot initial guess as Matrices belonging to SE(2)
function XR_guess = parse_poses_odom2SE(poses, num_poses, transitions, num_transitions)
    %I build XR_guess starting from the first pose, then applying transitions
    XR_guess = zeros(3,3,num_poses); %initializing the matrix
    [x,y,theta] = deal(poses(1).x, poses(1).y, poses(1).theta);

    %Rotation matrix part
    XR_guess(1,1,1) = cos(theta);
    XR_guess(1,2,1) = -sin(theta);
    XR_guess(2,1,1) = sin(theta);
    XR_guess(2,2,1) = cos(theta);

    %Offset Vector
    XR_guess(1,3,1) = x;
    XR_guess(2,3,1) = y;
    XR_guess(3,3,1) = 1;

    %We use the odometry in order to calculate next poses
    for i=1:num_transitions
        curr_transition = transitions(i);
        t_x = curr_transition.v(1);
        t_y = curr_transition.v(2);
        t_theta = curr_transition.v(3);
        T = v2t([t_x,0, t_theta]);
        XR_guess(:,:,i+1) = XR_guess(:,:,i)* T;
    endfor
endfunction

