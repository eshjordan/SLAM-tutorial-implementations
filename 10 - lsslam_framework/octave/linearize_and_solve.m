% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    i = edge.fromIdx:edge.fromIdx+2;
    j = edge.toIdx:edge.toIdx+2;
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(i);  % the first robot pose
    x2 = g.x(j);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
##    t1 = time();
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);
##    t2 = time();


    % TODO: compute and add the term to H and b
    b(i) += A'*edge.information*e;
    b(j) += B'*edge.information*e;
    
    H(i, i) += A'*edge.information*A;
    H(i, j) += A'*edge.information*B;
    H(j, i) += B'*edge.information*A;
    H(j, j) += B'*edge.information*B;
    

    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(i, i) += eye(3);
      
      needToAddPrior = false;
    end
    
##    t3 = time();
##    printf("linearize_pose_pose_constraint: %f\n", t2-t1);
##    printf("pose_landmark_state_update: %f\n", t3-t2);

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    i = edge.fromIdx:edge.fromIdx+2;
    j = edge.toIdx:edge.toIdx+1
    
    x = g.x(i);  % the robot pose
    l = g.x(j);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
##    t1 = time();
    [e, A, B] = linearize_pose_landmark_constraint(x, l, edge.measurement);
##    t2 = time();


    % TODO: compute and add the term to H and b
    b(i) += A'*edge.information*e;
    b(j) += B'*edge.information*e;
    
    H(i, i) += A'*edge.information*A;
    H(i, j) += A'*edge.information*B;
    H(j, i) += B'*edge.information*A;
    H(j, j) += B'*edge.information*B;

##    t3 = time();
##    printf("linearize_pose_landmark_constraint: %f\n", t2-t1);
##    printf("pose_landmark_state_update: %f\n", t3-t2);

  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
##t1 = time();
dx = -H\b;
##t2 = time();
##printf("solve: %f\n", t2-t1);

end
