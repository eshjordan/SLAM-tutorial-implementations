more off;
clear all;
close all;

addpath('tools');
##dbstop("linearize_and_solve", 40)
% load the graph into the variable g
% only leave one line uncommented

% simulation datasets
load ../data/simulation-pose-pose.dat
%load ../data/simulation-pose-landmark.dat

% real-world datasets
%load ../data/intel.dat
%load ../data/dlr.dat

% plot the initial state of the graph
plot_graph(g, 0);
init_err = compute_global_error(g);
prev_err = init_err;
printf('Initial error %f\n', init_err);

% the number of iterations
numIterations = 100;

% maximum allowed dx
EPSILON = 10^-4;

% Error
err = 0;

% carry out the iterations
for i = 1:numIterations
  printf('Performing iteration %d\n', i);

##  t1 = time();
  dx = linearize_and_solve(g);
  
  % TODO: apply the solution to the state vector g.x
  g.x += dx;
  [poses, landmarks] = get_poses_landmarks(g);
  g.x(poses + 2) = normalize_angle(g.x(poses + 2));

  % plot the current state of the graph
##  if(mod(i, 10) == 0)
  plot_graph(g, i);
##  endif

##  t2 = time();
  err = compute_global_error(g);
##  t3 = time();

  % Print current error
  printf('Current error: %f\n', err);
  printf('Prev delta:    %f\n', err - prev_err);
  printf('Init delta:    %f\n', err - init_err);
  prev_err = err;
##  printf('linearize_and_solve: %f\n', t2-t1);
##  printf('compute_global_error: %f\n', t3-t2);

  % TODO: implement termination criterion as suggested on the sheet
  if (dx < EPSILON)
    break;
  endif

end

printf('Final error %f\n', err);
