% 3D environment ONR
% Camilo Ordonez, October 13, 2023

clear all
clc
close all


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
ax = gca;

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



% Add pier and rail

% Load the STL file and extract the vertices and faces
%stlFile1 = 'pier_simple.STL';
stlFile1 = 'base.stl';

stlFile2 = 'Rail.stl';
stlFile3 = 'Drift_Wood.stl';

pier = stlread(stlFile1);
vertices_pier = pier.Points;
faces_pier = pier.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 1;

% Apply scaling to the vertices and translation
scaledVertices_pier = [2+ 7.5 7.5 10] + scalingFactor * vertices_pier;


rail = stlread(stlFile2);
vertices_rail = rail.Points;
faces_rail = rail.ConnectivityList;

scalingFactor = 1;

% Apply scaling to the vertices
scaledVertices_rail = [2 + 7.5 3.5 10] + scalingFactor * vertices_rail;

hold on
% Create a patch from the scaled vertices and original faces
h1 = trisurf(faces_pier, scaledVertices_pier(:, 1), scaledVertices_pier(:, 2), scaledVertices_pier(:, 3), ...
    'FaceColor', [100 100 100]/255, 'LineStyle', 'none', 'FaceAlpha', 1.0);
hold on
% Create a patch from the scaled vertices and original faces
h2 = trisurf(faces_rail, scaledVertices_rail(:, 1), scaledVertices_rail(:, 2), scaledVertices_rail(:, 3), ...
    'FaceColor', [186 140 99]/255, 'LineStyle', 'none', 'FaceAlpha', 1.0);


% Add Drift_wood
wood = stlread(stlFile3);
vertices_wood = wood.Points;
faces_wood = wood.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.007; % original
% scalingFactor = 0.01;

% Apply scaling to the vertices
scaledVertices_wood = [10 5 -0.2] + scalingFactor * vertices_wood;

hold on
% Create a patch from the scaled vertices and original faces
h3 = trisurf(faces_wood, scaledVertices_wood(:, 1), scaledVertices_wood(:, 2), scaledVertices_wood(:, 3), ...
    'FaceColor', 'k', 'LineStyle', 'none', 'FaceAlpha', 1.0);
hold on


% Add the robot
robot = stlread("MATLABBody.stl");
vertices_robot = robot.Points;
faces_robot = robot.ConnectivityList;

% Define a scaling factor to make the object smaller (e.g., 0.1 for 10% of original size)
scalingFactor = 0.005/3;

% Apply scaling to the vertices
scaledVertices_robot = [3 5 2] + scalingFactor * vertices_robot;

hold on
% Create a patch from the scaled vertices and original faces
h4 = trisurf(faces_robot, scaledVertices_robot(:, 1), scaledVertices_robot(:, 2), scaledVertices_robot(:, 3), ...
    'FaceColor', 'r', 'LineStyle', 'none', 'FaceAlpha', 1.0);
hold on




xlabel('X')
ylabel('Y')
zlabel('Z')




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

xMin = 0; xMax = 20;
yMin = 0; yMax = 20;
zMin = 0; zMax = 5;
min_coords = [xMin, yMin, zMin]; % Minimum coordinates
max_coords = [xMax, yMax, zMax]; % Maximum coordinates
resolution = 0.2; % Resolution (adjust as needed)

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
object_points = [object_points;scaledVertices_wood] % add drift wood

for i = 1:size(object_points, 1)
    % Check if the object point is within the grid
    if all(object_points(i, :) >= min_coords) && all(object_points(i, :) <= max_coords)
        % Convert object point to grid index
        grid_idx = round((object_points(i, :) - min_coords) / resolution) + 1;
        occupancy_grid(grid_idx(2), grid_idx(1), grid_idx(3)) = 1;
    end
end

% Visualize the occupancy grid with fully colored cubes for occupied voxels
figure;
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

keyboard()







figure()
plot3(X,Y,Z,'.')












stlFile1 = 'Drift_Wood.stl';
% %stlFile1 = 'kelp_ASM.STL';
% %stlFile1 = 'kelp_ASM(Straight) v3.STL';
%stlFile1 = 'MATLABBody.STL';
% 
% Create a patch from the STL file
h1 = trisurf(stlread(stlFile1),'FaceColor','[0.2 0.2 0.2]','LineStyle','none','FaceAlpha',1.0);

% Set up the figure and adjust its properties
%figure('WindowState', 'maximized', 'Color', [1 1 1]);

%ff = figure
%ff = open('MyRiverbed.fig')
%ff.WindowState = 'maximized';
%ff.Color=[1 1 1]
% Create axes
%axes1 = ff.Children;
axes1 = gca


% Create hgtransform
hgtransform1 = hgtransform('Parent',axes1);
rotz = makehgtform('zrotate',0);
rotx = makehgtform('xrotate',0);
roty = makehgtform('yrotate',0);
Rot = [1 0 0;0 1 0; 0 0 1]
scale = makehgtform('scale',0.1);

set(hgtransform1,'Matrix',scale);    


% Create patch
patch('Parent',hgtransform1,'Vertices',h1.Vertices,'LineStyle','none',...
    'Faces',h1.Faces,...
    'FaceVertexCData',h1.FaceVertexCData);

%set(axes1, 'XLim',[-1 10], 'YLim', [-5 10], 'ZLim', [-1.5 4])
set(axes1,'FontSize',24)




