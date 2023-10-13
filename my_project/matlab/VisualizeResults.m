stats = sbmpo_stats("../csv/stats.csv");
[paths, nodes] = sbmpo_results("../csv/nodes.csv");

goal_r = 1.0;
start_state = params.start_state;
goal = params.goal_state;

% Convert path states to points and plot
for p = 1:length(paths)

    %figure('Color', [1 1 1])
    hold on

    title(strcat("Results ", int2str(p)))
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("Z (m)")

    % Plot goal
    plot3(goal(1), goal(2), goal(3));

    % Plot all nodes
    nx = zeros(1, nodes(p).buffer_size);
    ny = zeros(1, nodes(p).buffer_size);
    nz = zeros(1, nodes(p).buffer_size);
    for n = 1:nodes(p).buffer_size
        node = nodes(p).nodes(n);
        nx(n) = node.state(1);
        ny(n) = node.state(2); 
        nz(n) = node.state(3);
    end
    % plot3(nx, ny, nz, 'ob', 'MarkerSize', 2, 'HandleVisibility', 'off')
    
    % Plot path
    px = zeros(1, paths(p).path_size);
    py = zeros(1, paths(p).path_size);
    pz = zeros(1, paths(p).path_size);
    for n = 1:paths(p).path_size
        node = paths(p).nodes(n);
        px(n) = node.state(1);
        py(n) = node.state(2);
        pz(n) = node.state(3);
    end
    plot3(px, py, pz, '-g', 'LineWidth', 5)
    plot3(px, py, pz, 'ob', 'MarkerSize', 5)
    
    axis([0 20 0 20 0 5])

end