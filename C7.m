% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    [row, col] = size(cspace);
    n_row = [1 row];
    n_col = [1 col];
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    padded_cspace = cspace;
    for row = 1:row
        for col = 1:col
            mat = cspace(row, col);
            if mat == 1
                for n = neighbors
                    n_x = row + n(1);
                    n_y = col + n(2);
                    if in_range(n_x, n_row) && in_range(n_y, n_col) && cspace(n_x, n_y) == 0
                        padded_cspace(n_x, n_y) = 1;
                    end
                end
            end
        end
    end
end
% 
% function eval = in_range(value, range)
%     left = range(1);
%     right = range(2);
%     eval = value >= left && value <= right;
% end