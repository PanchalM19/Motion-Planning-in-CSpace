% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 1 and 7 that denotes question
%                       number to run.
%        cspace -> Optional pre-computed configuration space to avoid
%                  re-computing it in questions C3-C7
% Output: cspace -> If cspace was computed, you can save it and pass it in
%                   on later calls to hw2_cspace to avoid re-computing it

function cspace = hw2_cspace(questionNum, cspace)

    close all;
    
    if nargin < 1
        error('Error: please enter a question number as a parameter');
    end
    
    % Set up obstacles as 2-D polygons
    p1 = polyshape([1.5 1.5 3.5 3.5], [1.4 0.7 0.7 1.4]);
    p2 = polyshape([9.1 8.3 8.3 9.8 9.8], [1.9 1.4 0.5 0.5 1.4]);
    p3 = polyshape([0.9 0.9 2.0 2.0 2.8 3.3 2.8 2.0 2.0], [7.2 3.7 3.7 4.8 4.8 5.5 6.2 6.2 7.2]);
    p4 = polyshape([9.0 9.0 8.3 8.3 9.0 9.0 10.0 10.0], [7.3 6.2 6.2 4.8 4.8 3.8 3.8 7.3]);
    p5 = polyshape([6.9 6.3 6.9], [7.1 5.6 5.6]);
    obstacles = [p1 p2 p3 p4 p5];
    
    % The 2-DOF 2-link rotational planar robot is encapsulated in a MATLAB cell
    robot = {};
    % Robot links are 2-D polygons as well
    % Pivot point of link 1, with respect to base frame (at origin)
    robot.pivot1 = [6.4; 2.5];
    % Pivot point of link 2, with respect to frame 1 (at pivot1)
    robot.pivot2 = [2.1; 0];
    % Corners of link 1 polygon, with respect to frame 1
    robot.link1 = [-1.2 -1.2 2.3 2.3; 0.5 -0.5 -0.4 0.4];
    % Corners of link 2 polygon, with respect to frame 2
    robot.link2 = [-0.3 -0.3 2.7 2.7; 0.4 -0.4 -0.2 0.2];
    
    q_start = [0.85; 0.9];
    q_goal = [3.05; 0.05];
    if nargin < 2
        cspace_resolution = 300;
    else
        cspace_resolution = size(cspace, 1);
    end
    q_grid = linspace(0, 2*pi, cspace_resolution);
    
    % ========== Question C1 ==========
    if questionNum == 1
        % Plot the robot in a given configuration, together with obstacles
        plot_obstacles(obstacles);
        % TODO: Implement this function
        C1(robot, q_start);
        C1(robot, q_goal);
    end
    
    % ========== Question C2 ==========
    if questionNum == 2
        % Compute the configuration space in a discrete grid at the
        % provided grid points
        % TODO: Implement this function
        cspace = C2(robot, obstacles, q_grid);
        % Visualize configuration space
        imshow(1 - cspace');
        set(gca, 'YDir', 'normal');
    end
    
    % ========== Question C3 ==========
    if questionNum == 3
        % If pre-computed configuration space is not provided,
        % compute the configuration space using C2
        if nargin < 2
            cspace = C2(robot, obstacles, q_grid);
        end
        % Compute distance transform from q_goal
        % TODO: Implement this function
        distances = C3(cspace, q_grid, q_goal);
        % Visualize distance transform
        imshow(distances', [min(min(distances)), max(max(distances))]);
        set(gca, 'YDir', 'normal');
    end
    
    % ========== Question 4 ==========
    if questionNum == 4
        % If pre-computed configuration space is not provided,
        % compute the configuration space using C2
        if nargin < 2
            cspace = C2(robot, obstacles, q_grid);
        end
        % Compute distance transform from q_goal
        distances = C3(cspace, q_grid, q_goal);
        % Find a path from q_start to q_goal using distance transform
        % TODO: Implement this function
        path = C4(distances, q_grid, q_start);
        % Visualize distance transform
        imshow(distances', [min(min(distances)), max(max(distances))]);
        hold on;
        % Visualize found path
        scatter(path(:,1), path(:,2), 'rs', 'MarkerFaceColor', 'r');
        set(gca, 'YDir', 'normal');
    end
    
    % ========== Question C5 ==========
    if questionNum == 5
        % If pre-computed configuration space is not provided,
        % compute the configuration space using C2
        if nargin < 2
            cspace = C2(robot, obstacles, q_grid);
        end
        % Compute distance transform from q_goal
        distances = C3(cspace, q_grid, q_goal);
        % Find a path from q_start to q_goal using distance transform
        path = C4(distances, q_grid, q_start);
        % Uncomment to visualize distance transform and found path
        % imshow(distances', [min(min(distances)), max(max(distances))]);
        % hold on;
        % scatter(path(:,1), path(:,2), 'rs', 'MarkerFaceColor', 'r');
        % set(gca, 'YDir', 'normal');
        plot_obstacles(obstacles);
        % Convert path in discretized grid into configuration-space path
        % TODO: Implement this function
        q_path = C5(q_grid, q_start, q_goal, path);
        % FPS (frames per second) controls time between frames
        fps = 4;
        % Use C1 to plot each robot configuration along the path
        for i = 1:size(q_path, 1)
            C1(robot, q_path(i,:)')
            pause(1.0 / fps);
        end
    end
    
    % ========== Question C6 ==========
    if questionNum == 6
        % If pre-computed configuration space is not provided,
        % compute the configuration space using C2
        if nargin < 2
            cspace = C2(robot, obstacles, q_grid);
        end
        % Compute distance transform from q_goal
        distances = C3(cspace, q_grid, q_goal);
        % Find a path from q_start to q_goal using distance transform
        path = C4(distances, q_grid, q_start);
        % Convert path in discretized grid into configuration-space path
        q_path = C5(q_grid, q_start, q_goal, path);
        % Uncomment to visualize path in workspace
        % for i = 1:size(q_path, 1)
        %     C1(robot, q_path(i,:)')
        % end
        plot_obstacles(obstacles);
        % Plot swept-volume collisions, if any
        % TODO: Implement this function
        num_collisions = C6(robot, obstacles, q_path);
        fprintf('Path contains %d collisions.\n', num_collisions);
    end
    
    % ========== Question C7 ==========
    if questionNum == 7
        % If pre-computed configuration space is not provided,
        % compute the configuration space using C2
        if nargin < 2
            cspace = C2(robot, obstacles, q_grid);
        end
        % Pad all c-space obstacles
        % TODO: Implement this function
        padded_cspace = C7(cspace);
        % Uncomment to visualize configuration space
        % imshow(1 - padded_cspace');
        % set(gca, 'YDir', 'normal');
        distances = C3(padded_cspace, q_grid, q_goal);
        path = C4(distances, q_grid, q_start);
        % Uncomment to visualize distance transform and found path
        % imshow(distances', [min(min(distances)), max(max(distances))]);
        % hold on;
        % scatter(path(:,1), path(:,2), 'rs', 'MarkerFaceColor', 'r');
        % set(gca, 'YDir', 'normal');
        plot_obstacles(obstacles);
        % Convert path in discretized grid into configuration-space path
        q_path = C5(q_grid, q_start, q_goal, path);
        % FPS (frames per second) controls time between frames
        fps = 4;
        % Use C1 to plot each robot configuration along the path
        for i = 1:size(q_path, 1)
            C1(robot, q_path(i,:)')
            pause(1.0 / fps);
        end
        plot_obstacles(obstacles);
        % Plot swept-volume collisions, if any
        num_collisions = C6(robot, obstacles, q_path);
        fprintf('Path contains %d collisions.\n', num_collisions);
    end
end
