clear all;
close all;

addpath('tools');

landmarks = read_world('../data/world.dat');
data = read_data('../data/sensor_data.dat');

motion_err = [0.2;0.2;0.05];
motion_cov = motion_err.*eye(3);
o_rb_err = [0.03;0.03];
observation_err = [
    o_rb_err(1)*sin(o_rb_err(2) + pi()/4);
    o_rb_err(1)*cos(o_rb_err(2) - pi()/4);
];
observation_cov = observation_err.*eye(2);

motion_noise = sqrtm(motion_cov)';
observation_noise = sqrtm(observation_cov)';
ai = motion_noise*motion_err;
ck = observation_noise*observation_err;

num_odometry = 0;       % N
num_observations = 0;   % K
num_landmarks = 0;      % M

d_x = 3;                % Dimensions of state
d_z = 2;                % Dimensions of measurement
d_l = 2;                % Dimensions of landmark

landmark_ids = [];      % Used to store previously seen landmarks

u = {};                 % Used to store control structs
z = {};                 % Used to store observation structs

dx = [0;0;0];
dl = [];
d = [dx; dl];           % Delta vector for state/landmark update


A = sparse(0, 0);       % System Matrix
r = 0;                  % Number of rows in A
c = 0;                  % Number of cols in A

b = [];            % Error vector
% I = [];

showGui = true; % plot to files instead

% Perform filter update for each odometry-observation pair read from the
% data file.
for t = 1:size(data.timestep, 2)
%for t = 1:50
    fprintf('timestep = %d\n', t);
    
    u{end+1} = data.timestep(t).odometry;
    z{end+1} = data.timestep(t).sensor;
    
    dx_prev = dx(end-2:end);
    dx_curr = motion_function(dx_prev, [u{t}.r1;u{t}.t;u{t}.r2]);
    dx = [dx; dx_curr];
    
    num_new_observations = length(z{end});
    
    for j = 1:num_new_observations
        % If landmark not previously seen
        if ~ismember(z{end}(j).id, landmark_ids)
            % Add to the list of observed landmarks
            landmark_ids = [landmark_ids, z{end}(j).id];
            
            % Add initial landmark position to state vector
            dl = [dl; observation_function(dx_curr, [z{end}(j).range;z{end}(j).bearing], false)];
        end
    end
    
    num_new_landmarks = length(landmark_ids) - num_landmarks;

    % Update count of variables
    num_odometry = num_odometry + 1;
    num_observations = num_observations + num_new_observations;
    num_landmarks = num_landmarks + num_new_landmarks;

    % Increase number of rows and columns by height and width of observation Jacobian
    c = c + 3*num_new_observations;
    r = r + 2*num_new_observations;

    % Increase number of columns by width of landmark Jacobian
    c = c + 2*num_new_landmarks;
    
    [F, G] = motion_jacobian(dx_prev, [u{t}.r1;u{t}.t;u{t}.r2]);
    
    TL = A(1:3*num_odometry,1:3*num_odometry);
    TM = zeros(3*num_odometry, 3);
    TR = zeros(3*num_odometry, 2*num_landmarks);
    M = zeros(3, 3*num_odometry + 3 + 2*num_landmarks);
    M(:, 3*(num_odometry-1)+1:3*num_odometry+3) = [motion_noise*F, motion_noise*G];
    BL = A(3*num_odometry+1:end,1:3*num_odometry);
    BM = zeros(2*num_observations, 3);
    BR = A(3*num_odometry+1:end,3*num_odometry+1:end);
    
    A = [
        TL, TM, TR;
        M;
        BL, BM, BR;
    ];
    
    b = [b;ai];

    if num_new_observations > 0
        [n_rows, n_cols] = size(A);
        
        if num_new_landmarks > 0
            A = [
                A, zeros(n_rows, 2*num_new_landmarks);
                zeros(2*num_new_observations, n_cols + 2*num_new_landmarks);
            ];
        else
            A = [
                A;
                zeros(2*num_new_observations, n_cols);
            ];
        end

        b = [b;repmat(ck, num_new_observations, 1)];
        
        l_start_col = n_cols - 2*num_landmarks + 1;
        for j = 1:num_new_observations
            l_id = find(landmark_ids == z{end}(j).id);

            add_r = n_rows+2*j-1:n_rows+2*j;
            add_c1 = l_start_col-3:l_start_col-1;
            add_c2 = l_start_col+2*(l_id-1):l_start_col+2*l_id-1;

            [H, J] = observation_jacobian(dx_curr, [z{end}(j).range;z{end}(j).bearing], true);
            A(add_r, add_c1) = observation_noise*H;
            A(add_r, add_c2) = observation_noise*J;
        end
    end
    % Generate visualization plots of the current state of the filter
%     plot_state(particles, landmarks, t, data.timestep(t).sensor, showGui);

    % Resample the particle set
%     particles = resample(particles);

    % spy(A);
    
    num_odometry = num_odometry+1;
    num_observations = num_observations + num_new_observations;
    num_landmarks = num_landmarks + num_new_landmarks;
    
end
