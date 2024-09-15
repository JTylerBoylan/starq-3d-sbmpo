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
    plot3(nx, ny, nz, 'ob', 'MarkerSize', 2, 'HandleVisibility', 'off')
    
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
    
    axis([0 5 0 5 0 1])

end


%% Voxel Grid

% I had to do some interpolation as the stl for the pier had very few
% points

[ind, vals] = find(scaledVertices_pier(:, 3)== 0); % foot print of the pilars.
X_pilars = scaledVertices_pier(ind, 1);
Y_pilars = scaledVertices_pier(ind, 2);
Z_pilars = scaledVertices_pier(ind, 3);

water_level = 4;
new_points = [];
for it = 1:numel(Z_pilars)
    for height = 0:0.05:water_level;
        new_point = [X_pilars(it) Y_pilars(it) height];
        new_points = [new_points; new_point]; 
    end
end

pilar_points = new_points;

Xp = pilar_points(:,1);
Yp = pilar_points(:,2);
Zp = pilar_points(:,3);


% Add points to the pier mesh from the ground upto the water level

xMin = 0; xMax = 5;
yMin = 0; yMax = 5;
zMin = 0; zMax = 1;
min_coords = [xMin, yMin, zMin]; % Minimum coordinates
max_coords = [xMax, yMax, zMax]; % Maximum coordinates
resolution = 0.1; % Resolution (adjust as needed)

% Create the 3D grid
x = min_coords(1):resolution:max_coords(1);
y = min_coords(2):resolution:max_coords(2);
z = min_coords(3):resolution:max_coords(3);
[X, Y, Z] = meshgrid(x, y, z);

% Initialize the occupancy grid with zeros
occupancy_grid = zeros(size(X));

% Set the voxels that intersect the object to 1
% Append all points that correspond to osbtacles
object_points = [Xp, Yp, Zp]; % pilar object points
object_points = [object_points;scaledVertices_wood]; % add drift wood
% object_points = scaledVertices_wood; % add drift wood

for i = 1:size(object_points, 1)
    % Check if the object point is within the grid
    if all(object_points(i, :) >= min_coords) && all(object_points(i, :) <= max_coords)
        % Convert object point to grid index
        grid_idx = round((object_points(i, :) - min_coords) / resolution) + 1;
        occupancy_grid(grid_idx(2), grid_idx(1), grid_idx(3)) = 1;
    end
end

% Visualize the occupancy grid with fully colored cubes for occupied voxels
axis equal;
hold on;

for i = 1:numel(X)
    if occupancy_grid(i) == 1
        % Define the vertices for a colored cube
        cube_size = resolution / 2;
        x_center = X(i);
        y_center = Y(i);
        z_center = Z(i);

        % Define the 8 vertices of the cube
        vertices = [
            x_center - cube_size, y_center - cube_size, z_center - cube_size;
            x_center + cube_size, y_center - cube_size, z_center - cube_size;
            x_center + cube_size, y_center + cube_size, z_center - cube_size;
            x_center - cube_size, y_center + cube_size, z_center - cube_size;
            x_center - cube_size, y_center - cube_size, z_center + cube_size;
            x_center + cube_size, y_center - cube_size, z_center + cube_size;
            x_center + cube_size, y_center + cube_size, z_center + cube_size;
            x_center - cube_size, y_center + cube_size, z_center + cube_size;
        ];

        % Define the six faces of the cube
        faces = [
            1 2 3 4;
            5 6 7 8;
            1 2 6 5;
            2 3 7 6;
            3 4 8 7;
            4 1 5 8;
        ];

        % Calculate the color based on the Z-coordinate
        color = [0, 0, Z(i) / zMax];

        % Create and plot the six faces of the cube
        for j = 1:6
            patch('Vertices', vertices, 'Faces', faces(j, :), 'FaceColor', color, 'EdgeColor', 'none');
        end
    end
end

xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Occupancy Grid with Colored Cubes');

hold off;
grid on