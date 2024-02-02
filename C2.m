% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
     [~, x] = size(q_grid);
     %Make a zero matrix have dim as q_grid
    cspace = zeros(x);
    
    %Need to use nested for loop to go through each row and column
    for row = 1:x
        for col = 1:x
            [poly1, poly2, ~, ~] = q2poly(robot, [q_grid(row) ; q_grid(col)]);
            %To avoid obstacle
            for to_avoid = obstacles
                link1_ob = intersect(poly1, to_avoid);
                link2_ob = intersect(poly2, to_avoid);
                
                %when link position coincides with obstacle
                if link1_ob.NumRegions > 0 || link2_ob.NumRegions > 0
                    cspace(row, col) = 1;
                    break;
                end
            end
            
        end
    end
end