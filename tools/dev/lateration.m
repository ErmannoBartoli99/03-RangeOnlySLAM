% This function compute the triangularization of the position of a landmark
% given at least 3 ranges observations and their respectively robot positions
%
% INPUT
% XR: Matrices of robot poses belonging to SE(2)
% observations: set of all the observation. each observation is from the pose -> pose_id and to landmark -> landmark_id
%
% OUTPUT:
% landmark_pos: position of landmark
function landmark_pos = lateration(XR, observations)
    
    % 3 steps
    % 1) Taking the last elements from both poses and ranges
    % 2) From every pose, we subtract the last one
    % 3) We solve the problem with a regularization term

    %Step 1
    xlast = XR(1,3,length(XR));
    ylast = XR(2,3,length(XR));
    rlast = observations(length(XR));

    %Initialize A and B for step 2
    n_poses = length(XR);
    A = zeros(n_poses - 1, 2);
    b = zeros(n_poses - 1, 1);

    %Step 2
    for i = 1:n_poses-1
        xi = XR(1,3,i);
        yi = XR(2,3,i);
        ri = observations(i);
        A(i,:) = 2*[(xi-xlast) (yi-ylast)];
        b(i) = (xi^2 - xlast^2) + (yi^2 - ylast^2) + (rlast^2 - ri^2);
    endfor

    %Step 3
    regularizator = eye(2) *1e-2;
    landmark_pos = (A' * A + regularizator)\A'*b;
endfunction