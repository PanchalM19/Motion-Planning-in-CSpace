% Input: q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
%        q_goal -> 2x1 vector denoting the goal configuration
%        path -> Mx2 matrix containing a collision-free path from q_start
%                to q_goal (as computed in C3, embedded in distances).
%                The entries of path are grid cell indices, i.e., integers
%                between 1 and N. The first row is the grid cell containing
%                q_start, the final row is the grid cell containing q_goal.
% Output: q_path -> Mx2 matrix containing a collision-free path from
%                   q_start to q_goal. Each row in q_path is a robot
%                   configuration. The first row should be q_start,
%                   the final row should be q_goal.

function q_path = C5(q_grid, q_start, q_goal, path)
    [M, ~] = size(path);
    q_path = path;
    q_path(1,:) = q_start;
    q_path(M,:) = q_goal;
    
    for a = 2:M-1
        s_a = path(a,:);
        q_path(a,:) = [q_grid(s_a(1)) q_grid(s_a(2))];
    end
end