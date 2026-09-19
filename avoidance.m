clear;
clc;
close all;

%% =========================================================
% ADAPTIVE PATH PLANNING - PHASE 2
% 3D Obstacle Detection + Automatic Avoidance
% No Automated Driving Toolbox required
%% =========================================================

%% SETTINGS

roadLength = 120;
roadWidth  = 12;

egoSpeed = 30;          % km/h
detectionRange = 25;    % metres

obstacleX = 55;
obstacleY = 0;

safeOffset = -3.5;      % Move sideways to avoid obstacle

%% CREATE FIGURE

fig = figure( ...
    'Name','Adaptive Path Planning and Collision Avoidance', ...
    'NumberTitle','off', ...
    'Color',[0.92 0.92 0.92]);

ax = axes(fig);

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');

xlim(ax,[0 70]);
ylim(ax,[-15 15]);
zlim(ax,[0 10]);

xlabel(ax,'Road Length (m)');
ylabel(ax,'Road Width (m)');
zlabel(ax,'Height (m)');

view(ax,40,24);

%% =========================================================
% GROUND
%% =========================================================

patch(ax, ...
    [0 roadLength roadLength 0], ...
    [-15 -15 15 15], ...
    [0 0 0 0], ...
    [0.30 0.55 0.25], ...
    'EdgeColor','none');

%% =========================================================
% ROAD
%% =========================================================

patch(ax, ...
    [0 roadLength roadLength 0], ...
    [-roadWidth/2 -roadWidth/2 roadWidth/2 roadWidth/2], ...
    [0.05 0.05 0.05 0.05], ...
    [0.23 0.23 0.23], ...
    'EdgeColor','none');

%% Road edges

plot3(ax,[0 roadLength], ...
    [-roadWidth/2 -roadWidth/2], ...
    [0.08 0.08], ...
    'w','LineWidth',2);

plot3(ax,[0 roadLength], ...
    [roadWidth/2 roadWidth/2], ...
    [0.08 0.08], ...
    'w','LineWidth',2);

%% =========================================================
% ROAD-SIDE OBJECTS
%% =========================================================

for x = 10:15:115

    createCuboid(ax, ...
        [x -10 1], ...
        1.5,1.5,2,0, ...
        [0.40 0.25 0.10]);

    createCuboid(ax, ...
        [x+5 10 1], ...
        1.5,1.5,2,0, ...
        [0.40 0.25 0.10]);

end

%% =========================================================
% OBSTACLE VEHICLE
%% =========================================================

createCuboid(ax, ...
    [obstacleX obstacleY 0.7], ...
    4.5,2,1.4,0, ...
    [0.85 0.1 0.1]);

createCuboid(ax, ...
    [obstacleX obstacleY 1.55], ...
    2.5,1.7,0.7,0, ...
    [0.45 0.05 0.05]);

text(ax, ...
    obstacleX,obstacleY,3, ...
    'OBSTACLE', ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center');

%% =========================================================
% GENERATE ADAPTIVE SAFE PATH
%% =========================================================

pathX = 5:0.35:110;

pathY = zeros(size(pathX));

% Automatically calculate avoidance zone based on obstacle position

avoidStart  = obstacleX - 20;
moveOutEnd  = obstacleX - 7;

returnStart = obstacleX + 8;
returnEnd   = obstacleX + 23;

for i = 1:length(pathX)

    x = pathX(i);

    %% Move sideways before obstacle
    if x >= avoidStart && x < moveOutEnd

        t = (x-avoidStart)/(moveOutEnd-avoidStart);

        pathY(i) = safeOffset * smoothStep(t);

    %% Stay beside obstacle
    elseif x >= moveOutEnd && x <= returnStart

        pathY(i) = safeOffset;

    %% Return to original path
    elseif x > returnStart && x <= returnEnd

        t = (x-returnStart)/(returnEnd-returnStart);

        pathY(i) = safeOffset * (1-smoothStep(t));

    else

        pathY(i) = 0;

    end

end

%% Show planned adaptive path

plot3(ax, ...
    pathX, ...
    pathY, ...
    0.15*ones(size(pathX)), ...
    '--', ...
    'LineWidth',2);

%% =========================================================
% CREATE EGO VEHICLE
%% =========================================================

egoX = pathX(1);
egoY = pathY(1);

carLength = 4.5;
carWidth  = 2;
carHeight = 1.3;

egoBody = createCuboid(ax, ...
    [egoX egoY carHeight/2], ...
    carLength,carWidth,carHeight,0, ...
    [0.05 0.35 0.95]);

egoRoof = createCuboid(ax, ...
    [egoX egoY 1.55], ...
    2.5,1.7,0.7,0, ...
    [0.05 0.18 0.45]);

%% =========================================================
% RUN SIMULATION
%% =========================================================

for i = 1:length(pathX)

    egoX = pathX(i);
    egoY = pathY(i);

    %% Determine vehicle angle

    if i < length(pathX)

        dx = pathX(i+1) - pathX(i);
        dy = pathY(i+1) - pathY(i);

    else

        dx = pathX(i) - pathX(i-1);
        dy = pathY(i) - pathY(i-1);

    end

    yaw = atan2(dy,dx);

    %% Move ego vehicle

    bodyVertices = cuboidVertices( ...
        [egoX egoY carHeight/2], ...
        carLength,carWidth,carHeight,yaw);

    roofVertices = cuboidVertices( ...
        [egoX egoY 1.55], ...
        2.5,1.7,0.7,yaw);

    set(egoBody,'Vertices',bodyVertices);
    set(egoRoof,'Vertices',roofVertices);

    %% =====================================================
    % OBSTACLE DETECTION
    %% =====================================================

    distance = sqrt( ...
        (obstacleX-egoX)^2 + ...
        (obstacleY-egoY)^2);

    %% =====================================================
    % COLLISION RISK + DECISION
    %% =====================================================

    if egoX > obstacleX + 10

        risk = 'SAFE';
        action = 'RETURNING TO PATH';

    elseif distance <= 8

        risk = 'HIGH';
        action = 'AVOIDING OBSTACLE';

    elseif distance <= detectionRange

        risk = 'MEDIUM';
        action = 'PATH REPLANNING';

    else

        risk = 'SAFE';
        action = 'GO STRAIGHT';

    end

    %% =====================================================
    % DASHBOARD
    %% =====================================================

    title(ax, sprintf( ...
        ['ADAPTIVE PATH PLANNING SYSTEM\n' ...
         'Speed: %.0f km/h   |   Distance: %.1f m   |   ' ...
         'Risk: %s   |   Action: %s'], ...
        egoSpeed,distance,risk,action), ...
        'FontSize',12);

    %% Camera follows ego vehicle

    if egoX > 25

        xlim(ax,[egoX-25 egoX+35]);

    end

    drawnow;

    pause(0.025);

end

title(ax, ...
    'DESTINATION REACHED - COLLISION AVOIDED', ...
    'FontSize',14, ...
    'FontWeight','bold');

disp('-------------------------------------------');
disp('Adaptive Path Planning Completed');
disp('Obstacle detected successfully');
disp('Collision avoided successfully');
disp('Destination reached');
disp('-------------------------------------------');


%% =========================================================
% FUNCTIONS
%% =========================================================

function value = smoothStep(t)

    % Creates smooth steering movement

    value = t.^2 .* (3 - 2*t);

end


function h = createCuboid(ax,center,L,W,H,yaw,color)

    vertices = cuboidVertices(center,L,W,H,yaw);

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


function vertices = cuboidVertices(center,L,W,H,yaw)

    x = center(1);
    y = center(2);
    z = center(3);

    localVertices = [
        -L/2 -W/2 -H/2
         L/2 -W/2 -H/2
         L/2  W/2 -H/2
        -L/2  W/2 -H/2

        -L/2 -W/2  H/2
         L/2 -W/2  H/2
         L/2  W/2  H/2
        -L/2  W/2  H/2
    ];

    R = [
        cos(yaw) -sin(yaw) 0
        sin(yaw)  cos(yaw) 0
        0         0        1
    ];

    vertices = localVertices * R';

    vertices(:,1) = vertices(:,1) + x;
    vertices(:,2) = vertices(:,2) + y;
    vertices(:,3) = vertices(:,3) + z;

end