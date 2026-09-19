clear;
clc;
close all;

%% ============================================
% ADAPTIVE PATH PLANNING
% PHASE 1 - CUSTOM 3D DRIVING ENVIRONMENT
% No Automated Driving Toolbox required
% =============================================

%% Create figure
fig = figure('Name','Adaptive Path Planning - 3D Simulation', ...
             'NumberTitle','off');

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');

axis(ax,'equal');
xlim(ax,[0 120]);
ylim(ax,[-15 15]);
zlim(ax,[0 12]);

xlabel(ax,'Road Length (m)');
ylabel(ax,'Road Width (m)');
zlabel(ax,'Height (m)');

title(ax,'Adaptive Path Planning - Unstructured Road');

view(ax,38,22);

%% ============================================
% CREATE GROUND
%% ============================================

patch(ax, ...
    [0 120 120 0], ...
    [-15 -15 15 15], ...
    [0 0 0 0], ...
    [0.25 0.55 0.25], ...
    'EdgeColor','none');

%% ============================================
% CREATE ROAD
%% ============================================

roadWidth = 12;

patch(ax, ...
    [0 120 120 0], ...
    [-roadWidth/2 -roadWidth/2 roadWidth/2 roadWidth/2], ...
    [0.05 0.05 0.05 0.05], ...
    [0.20 0.20 0.20], ...
    'EdgeColor','none');

%% Road boundaries

plot3(ax,[0 120],[-6 -6],[0.07 0.07], ...
    'LineWidth',2);

plot3(ax,[0 120],[6 6],[0.07 0.07], ...
    'LineWidth',2);

%% ============================================
% CREATE SIMPLE ROAD-SIDE OBJECTS
%% ============================================

for x = 10:15:115

    % Left side
    createCuboid(ax,[x -10 1], ...
        1.2,1.2,2,[0.45 0.30 0.15]);

    % Right side
    createCuboid(ax,[x+5 10 1], ...
        1.2,1.2,2,[0.45 0.30 0.15]);

end

%% ============================================
% CREATE EGO VEHICLE
%% ============================================

egoX = 5;
egoY = 0;

carLength = 4.5;
carWidth  = 2;
carHeight = 1.2;

egoBody = createCuboid(ax, ...
    [egoX egoY carHeight/2], ...
    carLength,carWidth,carHeight, ...
    [0.1 0.4 0.9]);

%% Car roof

egoRoof = createCuboid(ax, ...
    [egoX egoY 1.45], ...
    2.4,1.7,0.7, ...
    [0.15 0.25 0.45]);

%% ============================================
% CREATE ROAD PATH
%% ============================================

pathX = linspace(5,110,200);
pathY = zeros(size(pathX));

plot3(ax,pathX,pathY, ...
    0.12*ones(size(pathX)), ...
    '--', ...
    'LineWidth',2);

%% ============================================
% STATUS DISPLAY
%% ============================================

statusText = text(ax,5,-13,10, ...
    'SYSTEM STATUS: RUNNING', ...
    'FontSize',13, ...
    'FontWeight','bold');

speedText = text(ax,5,-13,8, ...
    'Speed: 0 km/h', ...
    'FontSize',12);

%% ============================================
% VEHICLE MOVEMENT
%% ============================================

speed = 30;       % km/h

for x = 5:0.4:110

    egoX = x;

    %% Calculate new body vertices
    bodyVertices = cuboidVertices( ...
        [egoX egoY carHeight/2], ...
        carLength,carWidth,carHeight);

    roofVertices = cuboidVertices( ...
        [egoX egoY 1.45], ...
        2.4,1.7,0.7);

    %% Move vehicle
    set(egoBody,'Vertices',bodyVertices);
    set(egoRoof,'Vertices',roofVertices);

    %% Update dashboard information
    set(speedText,'String', ...
        ['Speed: ' num2str(speed) ' km/h']);

    %% Camera follows vehicle
    if egoX > 25

        xlim(ax,[egoX-25 egoX+35]);

    end

    drawnow;

    pause(0.03);

end

set(statusText,'String', ...
    'SYSTEM STATUS: DESTINATION REACHED');

disp('3D Simulation Completed Successfully');


%% ============================================
% LOCAL FUNCTIONS
%% ============================================

function h = createCuboid(ax,center,L,W,H,color)

    vertices = cuboidVertices(center,L,W,H);

    faces = [
        1 2 3 4
        5 8 7 6
        1 5 6 2
        2 6 7 3
        3 7 8 4
        4 8 5 1
    ];

    h = patch(ax, ...
        'Vertices',vertices, ...
        'Faces',faces, ...
        'FaceColor',color, ...
        'EdgeColor',[0.1 0.1 0.1]);

end


function vertices = cuboidVertices(center,L,W,H)

    x = center(1);
    y = center(2);
    z = center(3);

    x1 = x - L/2;
    x2 = x + L/2;

    y1 = y - W/2;
    y2 = y + W/2;

    z1 = z - H/2;
    z2 = z + H/2;

    vertices = [
        x1 y1 z1
        x2 y1 z1
        x2 y2 z1
        x1 y2 z1

        x1 y1 z2
        x2 y1 z2
        x2 y2 z2
        x1 y2 z2
    ];

end