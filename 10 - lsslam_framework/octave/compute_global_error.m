% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    z = edge.measurement;
    Ri = [
      cos(x1(3)), -sin(x1(3));
      sin(x1(3)), cos(x1(3))
    ];
    Rij = [
      cos(z(3)), -sin(z(3));
      sin(z(3)), cos(z(3))
    ];
    e = [
      Rij'*(Ri'*(x2(1:2) - x1(1:2)) - z(1:2));
      x2(3) - x1(3) - z(3)
    ];
    Fx += e'*edge.information*e;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    z = edge.measurement;
    Ri = [
      cos(x(3)), -sin(x(3));
      sin(x(3)), cos(x(3))
    ];
    e = Ri'*(l - x(1:2)) - z;
    Fx += e'*edge.information*e;

  end

end
