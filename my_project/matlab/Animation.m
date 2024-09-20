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

view([145 15])
axis off
axis equal

[dist_paths, dist_nodes] = sbmpo_results("mindist_nodes.csv");
[energy_paths, energy_nodes] = sbmpo_results("minenergy_nodes.csv");

goal_r = 0.25;
start_state = [0.25; 2.5; 0];
goal = [4.75; 2.5; 0];
horizon_time = 2.0;

td = horizon_time:horizon_time:horizon_time*dist_paths.path_size;
te = horizon_time:horizon_time:horizon_time*energy_paths.path_size;

hold on

% Plot goal
plot3(goal(1), goal(2), goal(3));

% Plot all nodes
nx = zeros(1, dist_nodes.buffer_size);
ny = zeros(1, dist_nodes.buffer_size);
nz = zeros(1, dist_nodes.buffer_size);
for n = 1:dist_nodes.buffer_size
    node = dist_nodes.nodes(n);
    nx(n) = node.state(1);
    ny(n) = node.state(2); 
    nz(n) = node.state(3);
end
% plot3(nx, ny, nz, 'ob', 'MarkerSize', 2, 'HandleVisibility', 'off')

% Plot path
dist_px = zeros(1, dist_paths.path_size);
dist_py = zeros(1, dist_paths.path_size);
dist_pz = zeros(1, dist_paths.path_size);
for n = 1:dist_paths.path_size
    node = dist_paths.nodes(n);
    dist_px(n) = node.state(1);
    dist_py(n) = node.state(2);
    dist_pz(n) = node.state(3);
end
dist_plt_line = plot3(dist_px, dist_py, dist_pz, '--g', 'LineWidth', 2.5);
dist_plt_mark = plot3(dist_px, dist_py, dist_pz, 'ob', 'MarkerSize', 5);

% Plot path
energy_px = zeros(1, energy_paths.path_size);
energy_py = zeros(1, energy_paths.path_size);
energy_pz = zeros(1, energy_paths.path_size);
for n = 1:energy_paths.path_size
    node = energy_paths.nodes(n);
    energy_px(n) = node.state(1);
    energy_py(n) = node.state(2);
    energy_pz(n) = node.state(3);
end
energy_plt_line = plot3(energy_px, energy_py, energy_pz, '--g', 'LineWidth', 2.5);
energy_plt_mark = plot3(energy_px, energy_py, energy_pz, 'ob', 'MarkerSize', 5);

% calculate robot orientation
dpxd = diff(dist_px);
dpyd = diff(dist_py);
dpzd = diff(dist_pz);

ppitchd = atan2(dpzd, sqrt(dpxd.^2 + dpyd.^2));
pyawd = atan2(dpyd, dpxd);
prolld = pi/16 * sin(10 * td);

ppitchd = smoothdata(ppitchd);
pyawd = smoothdata(pyawd);

% calculate robot orientation
dpxe = diff(energy_px);
dpye = diff(energy_py);
dpze = diff(energy_pz);

ppitche = atan2(dpze, sqrt(dpxe.^2 + dpye.^2));
pyawe = atan2(dpye, dpxe);
prolle = pi/16 * sin(10 * te);

ppitche = smoothdata(ppitche);
pyawe = smoothdata(pyawe);

z_off = 0.1;

robot = stlread('stl/robot.stl');
robot_plt_dist = drawRobot([], robot, 0, 0, z_off, 0, 0, 0);
robot_plt_energy = drawRobot([], robot, 0, 0, z_off, 0, 0, 0);

% dist_plt_line = plot3(0,0,0,'--g','LineWidth', 2.5);
% energy_plt_line = plot3(0,0,0,'--g','LineWidth', 2.5);

if dist_paths.path_size < energy_paths.path_size
    size = energy_paths.path_size;
    t = te;
else
    size = dist_paths.path_size;
    t = td;
end

% animate
tic
vid = VideoWriter('animation.avi', 'Motion JPEG AVI');
vid.FrameRate = 5;
vid.Quality = 100;
vid.open()
pause(1.0)
for k = 2:size
    if (k <= dist_paths.path_size)
        xd_k = dist_px(k);
        yd_k = dist_py(k);
        zd_k = dist_pz(k) + z_off;
        roll_kd = prolld(k-1);
        pitch_kd = ppitchd(k-1);
        yaw_kd = pyawd(k-1);
        robot_plt_dist = drawRobot(robot_plt_dist, robot, xd_k, yd_k, zd_k, roll_kd, pitch_kd, yaw_kd);
        %set(dist_plt_line, 'xdata', dist_px(1:k), 'ydata', dist_py(1:k), 'zdata', dist_pz(1:k))
    end

    if (k <= energy_paths.path_size)
        xe_k = energy_px(k);
        ye_k = energy_py(k);
        ze_k = energy_pz(k) + z_off;
        roll_ke = prolle(k-1);
        pitch_ke = ppitche(k-1);
        yaw_ke = pyawe(k-1);
        robot_plt_energy = drawRobot(robot_plt_energy, robot, xe_k, ye_k, ze_k, roll_ke, pitch_ke, yaw_ke);
        %set(energy_plt_line, 'xdata', energy_px(1:k), 'ydata', energy_py(1:k), 'zdata', energy_pz(1:k))
    end

    writeVideo(vid, getframe(gcf))
    % pause(t(k) - toc);
end
vid.close()

function drawEnvironment()
% Define the dimensions of the box
boxWidth = 5;
boxLength = 5;
boxHeight = 1;

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
scalingFactor = 0.2;

% Apply scaling to the vertices and translation
scaledVertices_pier = [2.5 2.5 2] + scalingFactor * vertices_pier;

rail = stlread('stl/rail.stl');
vertices_rail = rail.Points;
faces_rail = rail.ConnectivityList;

% Apply scaling to the vertices
scaledVertices_rail = [2.5 2 2] + scalingFactor * vertices_rail;

hold on
% Create a patch from the scaled vertices and original faces
trisurf(faces_pier, scaledVertices_pier(:, 1), scaledVertices_pier(:, 2), scaledVertices_pier(:, 3), ...
    'FaceColor', [100 100 100]/255, 'LineStyle', 'none', 'FaceAlpha', 1.0);
hold on
% Create a patch from the scaled vertices and original faces
trisurf(faces_rail, scaledVertices_rail(:, 1), scaledVertices_rail(:, 2), scaledVertices_rail(:, 3), ...
    'FaceColor', [186 140 99]/255, 'LineStyle', 'none', 'FaceAlpha', 1.0);

end

function drawDriftWood()
% Add Drift_wood
wood = stlread('stl/driftwood.stl');
vertices_wood = wood.Points;
faces_wood = wood.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.00225; % original
% scalingFactor = 0.01;

% Apply scaling to the vertices
scaledVertices_wood = [2.5 1.0 0.16875] + scalingFactor * vertices_wood;

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

rotation = rotation_z * rotation_y * rotation_x * rotation_off;

vertices_robot = robot.Points;
faces_robot = robot.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.00225/3;

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