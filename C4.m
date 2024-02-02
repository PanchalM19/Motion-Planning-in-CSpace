% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    end_pos = find(distances==2);
    [~, N] = size(q_grid);
    g_x = mod(end_pos, N);
    g_y = fix(end_pos / N) + 1;
    
    tol = abs((q_grid(1) - q_grid(2))) / 2;
    check_x = find(abs(q_grid-q_start(1)) < tol);
    check_y = find(abs(q_grid-q_start(2)) < tol);
    path = [check_x check_y];
    
    while distances(check_x, check_y) ~= 2
        next_x = check_x;
        next_y = check_y;
        min = distances(next_x, next_y);
        diag = diagonal_dist(next_x, next_y, g_x, g_y);
        
        for n = neighbors
            n_x = n(1) + check_x;
            n_y = n(2) + check_y;
            n_dist = distances(n_x, n_y);
            if n_dist <= 1 %for unreachable condition
                continue
            end
            n_diag = diagonal_dist(n_x, n_y, g_x, g_y);
            if n_dist < min
                min = n_dist;
                next_x = n_x;
                next_y = n_y;
            elseif n_dist == min && n_diag < diag
                min = n_dist;
                diag = n_diag;
                next_x = n_x;
                next_y = n_y;
            end
        end
        check_x = next_x;
        check_y = next_y;
        path = [path; [check_x check_y]];
    end
end

%to use C4 independently, we define the distance function
function distance = diagonal_dist(xa, ya, xb, yb)%using 2 point distance formula
    x_dist = abs(xa - xb);
    y_dist = abs(ya - yb);
    distance = max(x_dist, y_dist);
end