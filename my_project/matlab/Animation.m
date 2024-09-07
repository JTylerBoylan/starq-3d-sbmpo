% 3D environment ONR
% Camilo Ordonez, October 13, 2023

clear
clc
close all

drawEnvironment();
xlabel('X')
ylabel('Y')
zlabel('Z')

drawPier();
drawDriftWood();

view([-30 30])
axis([0 15 0 15 0 7.5])

stats = sbmpo_stats("../csv/stats.csv");
[paths, nodes] = sbmpo_results("../csv/nodes.csv");

goal_r = 1.0;
start_state = [1; 8; 0];
goal = [15; 8; 0];
horizon_time = 0.1;

t = 0.1:0.1:0.1*paths.path_size;

hold on

% Plot goal
plot3(goal(1), goal(2), goal(3));

% Plot all nodes
nx = zeros(1, nodes.buffer_size);
ny = zeros(1, nodes.buffer_size);
nz = zeros(1, nodes.buffer_size);
for n = 1:nodes.buffer_size
    node = nodes.nodes(n);
    nx(n) = node.state(1);
    ny(n) = node.state(2); 
    nz(n) = node.state(3);
end
% plot3(nx, ny, nz, 'ob', 'MarkerSize', 2, 'HandleVisibility', 'off')

% Plot path
px = zeros(1, paths.path_size);
py = zeros(1, paths.path_size);
pz = zeros(1, paths.path_size);
for n = 1:paths.path_size
    node = paths.nodes(n);
    px(n) = node.state(1);
    py(n) = node.state(2);
    pz(n) = node.state(3);
end
% plot3(px, py, pz, '--g', 'LineWidth', 2.5)
% plot3(px, py, pz, 'ob', 'MarkerSize', 5)

% calculate robot orientation
dpx = diff(px);
dpy = diff(py);
dpz = diff(pz);

ppitch = atan2(dpz, sqrt(dpx.^2 + dpy.^2));
pyaw = atan2(dpy, dpx);
proll = pi/16 * sin(10 * t);

ppitch = smoothdata(ppitch);
pyaw = smoothdata(pyaw);

z_off = 0.35;

robot = stlread('stl/robot.stl');
robot_plt = drawRobot([], robot, 0, 0, z_off, 0, 0, 0);

% animate
tic
plt_line = plot3(0,0,0,'--g','LineWidth', 2.5);
for k = 2:paths.path_size
    x_k = px(k);
    y_k = py(k);
    z_k = pz(k) + z_off;
    roll_k = proll(k-1);
    pitch_k = ppitch(k-1);
    yaw_k = pyaw(k-1);
    robot_plt = drawRobot(robot_plt, robot, x_k, y_k, z_k, roll_k, pitch_k, yaw_k);
    set(plt_line, 'xdata', px(1:k), 'ydata', py(1:k), 'zdata', pz(1:k))
    pause(t(k) - toc);
end

function drawEnvironment()
% Define the dimensions of the box
boxWidth = 20;
boxLength = 20;
boxHeight = 5;

% Define the box vertices
vertices = [
    0, 0, 0;             % Vertex 1
    boxWidth, 0, 0;      % Vertex 2
    boxWidth, boxLength, 0; % Vertex 3
    0, boxLength, 0;      % Vertex 4
    0, 0, boxHeight;      % Vertex 5
    boxWidth, 0, boxHeight; % Vertex 6
    boxWidth, boxLength, boxHeight; % Vertex 7
    0, boxLength, boxHeight  % Vertex 8
    ];

% Define the box faces
faces = [
    1, 2, 3, 4; % Bottom face
    5, 6, 7, 8; % Top face
    1, 2, 6, 5; % Side face 1
    2, 3, 7, 6; % Side face 2
    3, 4, 8, 7; % Side face 3
    4, 1, 5, 8; % Side face 4
    ];


% Create the figure and add the box
figure('Color', 'white', 'WindowState', 'maximized');

% Create the patch for the box
hold on
patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'blue', 'FaceAlpha', 0.2, 'EdgeColor', 'none');

% Define the sand color for the bottom face
sandColor = [0.85, 0.75, 0.35]; % Sand color (RGB)

% Define the vertices of the square
x = [0,     boxWidth,         boxWidth,           0]; % X-coordinates of vertices
y = [0,       0,              boxLength,          boxLength]; % Y-coordinates of vertices
z = [0,       0,               0,         0]; % Z-coordinate of vertices (on the XY plane)

hold on
% Create the patch on the XY plane
patch(x, y, z, sandColor, 'FaceAlpha', 1.0); % 'b' for blue, adjust FaceAlpha for transparency

end

function drawPier()

pier = stlread('stl/base.stl');
vertices_pier = pier.Points;
faces_pier = pier.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 1;

% Apply scaling to the vertices and translation
scaledVertices_pier = [2+ 7.5 7.5 10] + scalingFactor * vertices_pier;

rail = stlread('stl/rail.stl');
vertices_rail = rail.Points;
faces_rail = rail.ConnectivityList;

scalingFactor = 1;

% Apply scaling to the vertices
scaledVertices_rail = [2 + 7.5 3.5 10] + scalingFactor * vertices_rail;

hold on
% Create a patch from the scaled vertices and original faces
trisurf(faces_pier, scaledVertices_pier(:, 1), scaledVertices_pier(:, 2), scaledVertices_pier(:, 3), ...
    'FaceColor', [100 100 100]/255, 'LineStyle', 'none', 'FaceAlpha', 0.5);
hold on
% Create a patch from the scaled vertices and original faces
trisurf(faces_rail, scaledVertices_rail(:, 1), scaledVertices_rail(:, 2), scaledVertices_rail(:, 3), ...
    'FaceColor', [186 140 99]/255, 'LineStyle', 'none', 'FaceAlpha', 0.5);

end

function drawDriftWood()
% Add Drift_wood
wood = stlread('stl/driftwood.stl');
vertices_wood = wood.Points;
faces_wood = wood.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.007; % original
% scalingFactor = 0.01;

% Apply scaling to the vertices
scaledVertices_wood = [10 5 -0.2] + scalingFactor * vertices_wood;

hold on
% Create a patch from the scaled vertices and original faces
trisurf(faces_wood, scaledVertices_wood(:, 1), scaledVertices_wood(:, 2), scaledVertices_wood(:, 3), ...
    'FaceColor', 'k', 'LineStyle', 'none', 'FaceAlpha', 1.0);

end

function plt = drawRobot(plt, robot, x, y, z, roll, pitch, yaw)

% Transform
translation = [x; y; z];

rotation_x = [1, 0, 0;
              0, cos(roll), -sin(roll);
              0, sin(roll), cos(roll)];
rotation_y = [cos(pitch), 0, -sin(pitch);
              0, 1, 0;
              sin(pitch), 0, cos(pitch)];
rotation_z = [cos(yaw), -sin(yaw), 0;
              sin(yaw), cos(yaw), 0;
              0, 0, 1];

rotation_off = [0, 0, 1;
                1, 0, 0;
                0, 1, 0];

rotation = rotation_off * rotation_z * rotation_y * rotation_x;

vertices_robot = robot.Points;
faces_robot = robot.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.005/3;

% Apply scaling to the vertices
scaledVertices_robot = translation + scalingFactor * rotation * vertices_robot.';

if ~isempty(plt)
    delete(plt)
end

% Create a patch from the scaled vertices and original faces
plt = trisurf(faces_robot, ...
    scaledVertices_robot(1,:), ...
    scaledVertices_robot(2,:), ...
    scaledVertices_robot(3,:), ...
    'FaceColor', 'r', 'LineStyle', 'none', 'FaceAlpha', 1.0);
end