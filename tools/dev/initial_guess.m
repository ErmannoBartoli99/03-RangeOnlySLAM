%This function computes the landmark positions at initial guess
%The computations is done given the robot poses and the observations
%
%INPUT
% XR_guess: it is the initial guess as a Matrix belonging to SE(2)
% id_first_pose: is the id of the first pose in the dataset
% num_poses: it is the number of the poses
% observations: It is the set of observations, each is the observation at a certain pose-> pose_id to a certain landmark -> landmark_id
% num_observations: it is the number of the observations
% num_landmarks: is the number of the KNOWN landmarks
%
%OUTPU
% XL_guess: It is the vector of the landmark positions at the initial guess
% associations(land and pose): It is the matrix which contains the associations between pose_ids and landmarks_ids
% Zl: it is the vector of landmark measures
% Zr: it is the vector of odometry measures
function [XL_guess, land_associations, pose_associations, Zl, Zr] = initial_guess(XR_guess, id_first_pose, poses, num_poses, num_transitions ,observations, transitions, num_observations, num_landmarks)
    
    XL_guess = [];
    land_associations = [];
    pose_associations = [];
    Zl = [];
    Zr  = [];
    observations_matrix = ones(num_landmarks, num_poses)* -1;
    
    %We have to build a map from index to the id of the landmark
    %and from the id of the landmark to the index
    idx2land_id = ones(num_landmarks, 1) * -1;
    land_id2idx = ones(1000,1) * -1;
    idx = 1;

    for i = 1:num_observations
      pose_idx = observations(i).pose_id - id_first_pose +1;
      for j = 1:length(observations(i).observation)
        obs_range = observations(i).observation(j).range;
        land_id = observations(i).observation(j).id;
        if land_id2idx(land_id) == -1
          land_id2idx(land_id) = idx;
          idx2land_id(idx) = land_id;
          idx = idx +1;
        endif
        observations_matrix(land_id2idx(land_id), pose_idx) = obs_range;
      endfor
    endfor

    %We have to triangularize with lateration method, then we're going to save
    %only the landmarks that have at least 3 measurements

    real_idx2land_id = zeros(num_landmarks,1);
    real_land_id2idx = zeros(1000,1);
    real_idx = 1;

    for i=1:num_landmarks
      p = [];
      r = [];
      for p_idx = 1:num_poses
        if observations_matrix(i, p_idx) != -1
          %Must save the index of the pose
          p(:,:, end+1) = XR_guess(:,:,p_idx);
          r(:,end+1) = observations_matrix(i,p_idx);
        endif
      endfor

      if length(r) > 2
        XL_guess(:, end+1) = lateration(p(:,:,2:end), r);
        landmark_id = idx2land_id(i);
        real_idx2land_id(real_idx) = landmark_id;
        real_land_id2idx(landmark_id) = real_idx;
        real_idx = real_idx + 1;
      endif
    endfor

    num_real_landmarks = real_idx -1;

    % In the last step we have to build the association matrix and measuerement vector

    for p_idx = 1:num_poses
      for l_idx = 1:num_real_landmarks
        landmark_id = real_idx2land_id(l_idx);
        obs_range = observations_matrix(land_id2idx(landmark_id), p_idx);
        if obs_range != -1
          Zl(:,end+1) = obs_range;
          land_associations(:,end+1) = [p_idx, l_idx];
        endif
      endfor
    endfor

    for p_idx = 1:num_transitions
      pose_i = find([poses.id] == transitions(i).id_from);
      pose_j = find([poses.id] == transitions(i).id_to);

      pose_associations(:,end+1) = [pose_i; pose_j];
      Zr(:, end+1) = [transitions(i).v(1); transitions(i).v(3)];
    endfor

endfunction