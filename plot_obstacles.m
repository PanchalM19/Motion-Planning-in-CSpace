% Input: obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles

function plot_obstacles(obstacles)
    % Plot the obstacles
    figure;
    xlim([0 10]);
    ylim([0 10]);
    axis square;
    hold on;
    for a = 1:length(obstacles)
        plot(obstacles(a), 'FaceColor', 'k');
    end
end