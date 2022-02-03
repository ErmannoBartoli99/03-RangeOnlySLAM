close all
clear
clc

addpath '../'
addpath 'tools'
addpath 'tools/g2o_wrapper'
addpath 'dataset'
source './tools/dev/data_parsing.m'
source './tools/dev/initial_guess.m'
source './tools/dev/lateration.m'
source './tools/dev/least_squares.m'

warning('off','all');

debug_mode = 1;

%The algorithm is composed by 3 phases:

%1) Generation of an initial guess of the position in the world frame of the landmark
%which is followed by a triangularization step in order to get a correct guess on the position of the landmark

%2) ICP algorithm: here we use range measuerements in order to converge both the pose of the robot and the
%position of the landmarks

%3) Final step: showing the results by generating a plot

%FIRST STEP


%Loading Datasets:
%The 2 files contains the poses and range observation, and the ground_truth
[landmarks_ground_truth, poses_ground_truth, transitions_ground_truth, observations_ground_truth] = loadG2o('slam2d_range_only_ground_truth.g2o');
[landmarks, poses, transitions, observations] = loadG2o('slam2d_range_only_initial_guess.g2o');

landmarks_ground_truth = landmarks_ground_truth(2:end);
poses_ground_truth = poses_ground_truth(2:end);
transitions_ground_truth = transitions_ground_truth(2:end);
observations_ground_truth = observations_ground_truth(2:end);

landmarks = landmarks(2:end);
poses = poses(2:end);
transitions = transitions(2:end);
observations = observations(2:end);

%Defininf some useful variables
num_poses = length(poses_ground_truth);
num_landmarks = length(landmarks_ground_truth);
num_transitions = length(transitions_ground_truth);
num_observations = length(observations_ground_truth);


% We can print the informations about the ground truth
if (debug_mode == 1)
    disp("Num of poses [ground truth]:");
    disp(num_poses);
    disp("Num of landmarks [ground truth]:");
    disp(num_landmarks);
    disp("Num of transitions [ground truth]:");
    disp(num_transitions);
    disp("Num of observations [ground truth]:");
    disp(num_observations);
endif

%We build the set of matrices containing rotation e translations
XR_true = poses_ground_truth2SE(poses_ground_truth, num_poses);
if(debug_mode == 1)
    disp("XR_true computed");
endif
XR_guess = parse_poses_odom2SE(poses, num_poses, transitions, num_transitions);
if(debug_mode == 1)
    disp("XR_guess computed");
endif

%INITIAL GUESS

%We compute the id of the first pose
id_first_pose = poses(1).id;
num_observations_ig = length(observations);
num_poses_ig = length(poses);
[XL_guess, land_associations, pose_associations, Zl, Zr] = initial_guess(XR_guess, id_first_pose, poses, num_poses_ig, num_transitions,observations, transitions, num_observations_ig, num_landmarks);
num_landmarks_ig = length(XL_guess);

XL_true = positionLandmark_ground_truth(landmarks_ground_truth, num_landmarks);
if(debug_mode == 1)
    disp("XL_true computed");
endif

num_iterations = 10;
damping = 0.001;
kernel_threshold = 1.0;

[XR, XL, chi_stats_l, chi_stats_r, num_inliers] = least_squares(XR_guess, XL_guess, Zl, Zr, land_associations, pose_associations, ...
                                     num_poses_ig, num_landmarks_ig, num_iterations, damping,kernel_threshold);



if(debug_mode == 1)
    disp("Least Squares Computed");
endif

% Plotting the results

%Figure one
figure(1);
hold on;
title("Landmarks positions");
axis("equal");
plot(XL_guess(1,:),XL_guess(2,:),'xr', "linewidth", 2);
plot(XL_true(1,:),XL_true(2,:), 'dg', "linewidth", 2);
plot(XL(1,:),XL(2,:), 'sb', "linewidth", 2);
legend("Initial guess","Groud truth", "Optimization");

%Figure two
figure(2);
hold on;
title("Robot trajectory");
axis("equal");
plot(squeeze(XR_guess(1,3,:)), squeeze(XR_guess(2,3,:)), 'r', 'linewidth', 2);
plot(squeeze(XR_true(1,3,:)), squeeze(XR_true(2,3,:)), 'g', 'linewidth', 2);
plot(squeeze(XR(1,3,:)), squeeze(XR(2,3,:)), 'b', 'linewidth', 2);
legend("Initial guess","Groud truth", "Optimization");

% Figure three
% This figure in particular, shows the trajectory assuming a good estimate of the calibration odometry of the robot
odom_cal = [-0.2 -2 0.20];
for i=1:num_poses
    XR_guess(:,:,i) = v2t(odom_cal) * XR_guess(:,:,i);
    XR(:,:,i) = v2t(odom_cal) * XR(:,:,i);
endfor

figure(3);
hold on;
title("Trajectory of the robot assuming a calibrated odometry");


plot(squeeze(XR_guess(1,3,:)), squeeze(XR_guess(2,3,:)), 'r', 'linewidth', 2);
plot(squeeze(XR_true(1,3,:)), squeeze(XR_true(2,3,:)), 'g', 'linewidth', 2);
plot(squeeze(XR(1,3,:)), squeeze(XR(2,3,:)), 'b', 'linewidth', 2);
legend("Initial guess","Ground truth","LS optimization");

figure(4);
hold on;
title("Chi evolution measurements");
plot(chi_stats_l);

figure(5);
hold on;
title("Chi evolution odometry");
plot(chi_stats_r);


if(debug_mode == 1)
    disp("Plots completed");
endif

pause();