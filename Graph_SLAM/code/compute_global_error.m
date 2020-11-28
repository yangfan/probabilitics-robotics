% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    X_i = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    X_j = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    % compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z_ij = v2t(edge.measurement);
    e_ij = t2v(invt(Z_ij)* (invt(X_i) * X_j));
    
    Fx += e_ij' * edge.information * e_ij;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    X = v2t(x);
    e_il = X(1:2,1:2)' * (l - x(1:2)) - edge.measurement;
%    e_il = (l - x(1:2)) - X(1:2,1:2) * edge.measurement;
    
    Fx += e_il' * edge.information * e_il;
  end

end
