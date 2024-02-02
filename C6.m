% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0; %to initialize
    for a = 1:(length(q_path) - 1)
        fig_1 = q_path(a,:);
        fig_2 = q_path(a+1,:);
        [poly1_fig1, poly2_fig1, ~, ~] = q2poly(robot, fig_1);
        [poly1_fig2, poly2_fig2, ~, ~] = q2poly(robot, fig_2);
        vol_1 = calculate_swept_vol(poly1_fig1, poly1_fig2);
        vol_2 = calculate_swept_vol(poly2_fig1, poly2_fig2);
        total_vol = union(vol_1, vol_2);
        for ob = obstacles
            if intersect(ob, total_vol).NumRegions > 0
                num_collisions = num_collisions + 1;
                C1(robot, fig_1');
                C1(robot, fig_2');
                plot(vol_1, 'FaceColor', 'r');
                plot(vol_2, 'FaceColor', 'b');
                break;
            end
        end
    end
end

function swept_volume = calculate_swept_vol(x, y)
    points = [x.Vertices; y.Vertices];
    c_hull = convhull([x.Vertices; y.Vertices]);
    swept_volume = polyshape(points(c_hull, 1), points(c_hull, 2));
end