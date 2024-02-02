% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    distances = cspace;
    tol = abs((q_grid(1) - q_grid(2))) / 2;
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    [max_row, max_col] = size(cspace);
    
    x = find(abs(q_grid-q_goal(1)) < tol);
    y = find(abs(q_grid-q_goal(2)) < tol);
    distances(x, y) = 2;
   
    check_xy = [x, y];
    [M, ~] = size(check_xy);
    while M > 0
        row = check_xy(1,:);
        row_x = row(1);
        row_y = row(2);
        row_dist = distances(row_x, row_y);
        check_xy(1,:) = [];
        for n = neighbors
            n_x = row_x + n(1);
            n_y = row_y + n(2);
            if ((n_x >= 1) && (n_x <= max_col)...
                && (n_y >= 1) && (n_y <= max_row)...
                && distances(n_x, n_y) == 0)
                check_xy = [check_xy; [n_x, n_y]];
                distances(n_x, n_y) = row_dist + 1;
            end
        end 
        [M, ~] = size(check_xy); 
    end
end