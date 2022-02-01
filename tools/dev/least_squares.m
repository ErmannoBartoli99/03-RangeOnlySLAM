source "tools/utilities/geometry_helpers_2d.m"

function [ e, Jr, Jl ] = errorAndJacobian(Xr, Xl, z)
    R = Xr(1:2,1:2);
    t = Xr(1:2,3);
    
    p_hat = R'*(Xl-t);
    z_hat = norm(p_hat);
    e = z_hat-z;

    Jr = zeros(1,3);
    J_icp = zeros(2,3);

    J_icp(1:2,1:2) = -R';
    J_icp(1:2,3) = R' * [0 1;-1 0] * Xl;
    Jr = (1/norm(p_hat))*p_hat'*J_icp;
    Jl = (1/norm(p_hat))*p_hat'*R';
endfunction

% Retrieves indices of poses/landmarks in the Hessian matrix
function [pos_matrix_idx, land_matrix_idx] = matrixIndex(pose_idx, landmark_idx, num_poses, num_landmarks)

    
    if pose_idx > num_poses || pose_idx <= 0
        pos_matrix_idx = -1;
    else
        pos_matrix_idx = 1 + (pose_idx-1)*3;
    endif


    if landmark_idx > num_landmarks || landmark_idx <= 0
        land_matrix_idx = -1;
    else
        land_matrix_idx = 1 + (num_poses)*3 + (landmark_idx-1)*2;
    endif
    
endfunction

function [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx)

    pose_dim = 3;
    landmark_dim = 2;

    for i=1:num_poses

        [pos_matrix_idx, _] = matrixIndex(i,0,num_poses, num_landmarks);
        dxr = dx(pos_matrix_idx: pos_matrix_idx+pose_dim-1);
        XR(:,:,i) = v2t(dxr)*XR(:,:,i);

    endfor

    for i=1:num_landmarks

        [_, land_matrix_idx] = matrixIndex(0,i,num_poses, num_landmarks);
        dxl = dx(land_matrix_idx:land_matrix_idx+landmark_dim-1,:);
        XL(:,i) += dxl;

    endfor

endfunction

function [XR, XL, chi_stats, num_inliers] = least_squares(XR, XL, Z, associations, num_poses, num_landmarks, num_iterations, damping, kernel_threshold)
    
    chi_stats = zeros(1, num_iterations);
    num_inliers = zeros(1, num_iterations);
    pose_dim = 3;
    landmark_dim = 2;

    %Size of the linear system
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    for i=1:num_iterations
        disp(['[Iteration Number] -> ', num2str(i)]);
        %Initializing matrix H and vector b
        % H has dim nxn
        % b has dim nx1
        H = zeros(system_size, system_size);
        b = zeros(system_size,1);
        for mes=1:length(Z)
            pose_idx = associations(1,mes);
            landmark_idx = associations(2, mes);
            z = Z(:,mes);
            Xr = XR(:,:,pose_idx);
            Xl = XL(:,landmark_idx);
            [e, Jr, Jl] = errorAndJacobian(Xr, Xl, z);
            chi = e' * e;

            if chi > kernel_threshold
                e *= sqrt(kernel_threshold/chi);
    
                chi = kernel_threshold;
            else
                num_inliers(i) ++;
            endif

            chi_stats(i) += chi;

            Hrr = Jr' * Jr;
            Hrl = Jr' * Jl;
            Hll = Jl' * Jl;
            br  = Jr' * e;
            bl  = Jl' * e;

            
            [pos_matrix_idx, land_matrix_idx] = matrixIndex(pose_idx, landmark_idx, num_poses, num_landmarks);

            H(pos_matrix_idx:pos_matrix_idx + pose_dim-1, pos_matrix_idx:pos_matrix_idx+pose_dim-1) += Hrr;
            H(pos_matrix_idx:pos_matrix_idx + pose_dim -1, land_matrix_idx:land_matrix_idx+landmark_dim-1) += Hrl;
            H(land_matrix_idx:land_matrix_idx+landmark_dim-1, land_matrix_idx:land_matrix_idx+landmark_dim-1) += Hll;
            H(land_matrix_idx:land_matrix_idx+landmark_dim-1, pos_matrix_idx:pos_matrix_idx + pose_dim -1) += Hrl';

            b(pos_matrix_idx:pos_matrix_idx + pose_dim -1) += br;
            b(land_matrix_idx:land_matrix_idx+landmark_dim-1) += bl;
        endfor

        H+= eye(system_size)*damping;
        dx = zeros(system_size,1);

        dx(pose_dim+1:end) = -(H(pose_dim+1:end, pose_dim+1:end)\b(pose_dim+1:end,1));
        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
    
    endfor


endfunction