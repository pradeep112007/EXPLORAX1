clear;
clc;
close all;

%% =========================================================
% PHASE 4 - DYNAMIC COLLISION PREDICTION
% Moving vehicle + TTC + Automatic Overtaking
% Basic MATLAB only
%% =========================================================

%% SETTINGS

roadLength = 140;
roadWidth  = 12;

dt = 0.05;                  % simulation timestep

egoX = 5;
egoY = 0;

egoSpeed = 8.33;            % 30 km/h

obstacleX = 42;
obstacleY = 0;

obstacleSpeed = 3.5;        % slower moving vehicle

safeLaneY = -3.5;

TTCwarning = 6;             % seconds
TTChigh = 3;                % seconds

%% =========================================================
% CREATE FIGURE
%% =========================================================

fig = figure( ...
    'Name','Dynamic Collision Prediction', ...
    'NumberTitle','off', ...
    'Color',[0.94 0.94 0.94]);

ax = axes(fig);

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');

xlim(ax,[0 75]);
ylim(ax,[-15 15]);
zlim(ax,[0 10]);

xlabel(ax,'Road Length (m)');
ylabel(ax,'Road Width (m)');
zlabel(ax,'Height (m)');

view(ax,40,22);

%% =========================================================
% GROUND
%% =========================================================

patch(ax, ...
    [0 roadLength roadLength 0], ...
    [-15 -15 15 15], ...
    [0 0 0 0], ...
    [0.25 0.55 0.25], ...
    'EdgeColor','none');

%% =========================================================
% ROAD
%% =========================================================

patch(ax, ...
    [0 roadLength roadLength 0], ...
    [-roadWidth/2 -roadWidth/2 ...
      roadWidth/2 roadWidth/2], ...
    [0.05 0.05 0.05 0.05], ...
    [0.22 0.22 0.22], ...
    'EdgeColor','none');

%% Road boundaries

plot3(ax,[0 roadLength],[-6 -6],[0.08 0.08], ...
    'w','LineWidth',2);

plot3(ax,[0 roadLength],[6 6],[0.08 0.08], ...
    'w','LineWidth',2);

%% =========================================================
% ROADSIDE OBJECTS
%% =========================================================

for x = 10:15:130

    createCuboid(ax,[x -10 1], ...
        1.3,1.3,2,0,[0.40 0.25 0.10]);

    createCuboid(ax,[x+5 10 1], ...
        1.3,1.3,2,0,[0.40 0.25 0.10]);

end

%% =========================================================
% CREATE EGO VEHICLE
%% =========================================================

egoBody = createCuboid(ax, ...
    [egoX egoY 0.65], ...
    4.5,2,1.3,0, ...
    [0.05 0.35 0.95]);

egoRoof = createCuboid(ax, ...
    [egoX egoY 1.55], ...
    2.5,1.7,0.7,0, ...
    [0.05 0.15 0.40]);

%% =========================================================
% CREATE MOVING OBSTACLE VEHICLE
%% =========================================================

obsBody = createCuboid(ax, ...
    [obstacleX obstacleY 0.65], ...
    4.5,2,1.3,0, ...
    [0.90 0.10 0.10]);

obsRoof = createCuboid(ax, ...
    [obstacleX obstacleY 1.55], ...
    2.5,1.7,0.7,0, ...
    [0.45 0.05 0.05]);

%% =========================================================
% DASHBOARD
%% =========================================================

dashboard = annotation(fig, ...
    'textbox', ...
    [0.19 0.82 0.62 0.14], ...
    'String','INITIALIZING', ...
    'FontSize',12, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center', ...
    'BackgroundColor',[1 1 1], ...
    'EdgeColor',[0.2 0.2 0.2]);

%% =========================================================
% VARIABLES
%% =========================================================

targetY = 0;

state = "FOLLOW";

overtakeStarted = false;
passedObstacle = false;

currentEgoSpeed = egoSpeed;

pathXhistory = [];
pathYhistory = [];

pathPlot = plot3(ax,nan,nan,nan, ...
    'LineWidth',2);

%% =========================================================
% SIMULATION
%% =========================================================

for frame = 1:1000

    %% Move obstacle

    obstacleX = obstacleX + obstacleSpeed*dt;

    %% -----------------------------------------------------
    % DISTANCE
    %% -----------------------------------------------------

    longitudinalDistance = obstacleX - egoX;

    relativeSpeed = ...
        currentEgoSpeed - obstacleSpeed;

    %% -----------------------------------------------------
    % TIME TO COLLISION
    %% -----------------------------------------------------

    if longitudinalDistance > 0 && ...
       relativeSpeed > 0

        TTC = longitudinalDistance / relativeSpeed;

    else

        TTC = inf;

    end

    %% =====================================================
    % DECISION MAKING
    %% =====================================================

    if state == "FOLLOW"

        if TTC < TTCwarning

            state = "PREPARE OVERTAKE";

        end

    end

    %% Start overtaking

    if state == "PREPARE OVERTAKE"

        targetY = safeLaneY;

        overtakeStarted = true;

        state = "OVERTAKING";

    end

    %% Check whether ego has passed obstacle

    if overtakeStarted && ...
       egoX > obstacleX + 7

        passedObstacle = true;

    end

    %% Return to original path

    if passedObstacle

        targetY = 0;
        state = "RETURNING";

    end

    %% Finished overtaking

    if state == "RETURNING" && ...
       abs(egoY) < 0.15

        egoY = 0;
        targetY = 0;

        state = "CRUISE";

    end

    %% =====================================================
    % RISK LEVEL
    %% =====================================================

    if TTC < TTChigh && ...
       state ~= "RETURNING" && ...
       state ~= "CRUISE"

        risk = "HIGH";

    elseif TTC < TTCwarning && ...
           state ~= "RETURNING" && ...
           state ~= "CRUISE"

        risk = "MEDIUM";

    else

        risk = "SAFE";

    end

    %% =====================================================
    % LATERAL STEERING
    %% =====================================================

    lateralError = targetY - egoY;

    maxLateralStep = 0.075;

    lateralStep = max( ...
        min(lateralError,maxLateralStep), ...
        -maxLateralStep);

    egoY = egoY + lateralStep;

    %% =====================================================
    % MOVE EGO FORWARD
    %% =====================================================

    egoX = egoX + currentEgoSpeed*dt;

    %% =====================================================
    % CALCULATE VEHICLE YAW
    %% =====================================================

    yaw = atan2(lateralStep, ...
        currentEgoSpeed*dt);

    %% =====================================================
    % UPDATE EGO VEHICLE
    %% =====================================================

    egoBodyVertices = cuboidVertices( ...
        [egoX egoY 0.65], ...
        4.5,2,1.3,yaw);

    egoRoofVertices = cuboidVertices( ...
        [egoX egoY 1.55], ...
        2.5,1.7,0.7,yaw);

    set(egoBody, ...
        'Vertices',egoBodyVertices);

    set(egoRoof, ...
        'Vertices',egoRoofVertices);

    %% =====================================================
    % UPDATE MOVING OBSTACLE
    %% =====================================================

    obsBodyVertices = cuboidVertices( ...
        [obstacleX obstacleY 0.65], ...
        4.5,2,1.3,0);

    obsRoofVertices = cuboidVertices( ...
        [obstacleX obstacleY 1.55], ...
        2.5,1.7,0.7,0);

    set(obsBody, ...
        'Vertices',obsBodyVertices);

    set(obsRoof, ...
        'Vertices',obsRoofVertices);

    %% =====================================================
    % STORE SAFE PATH
    %% =====================================================

    pathXhistory(end+1) = egoX;
    pathYhistory(end+1) = egoY;

    set(pathPlot, ...
        'XData',pathXhistory, ...
        'YData',pathYhistory, ...
        'ZData',0.12*ones(size(pathXhistory)));

    %% =====================================================
    % DISPLAY TTC
    %% =====================================================

    if isinf(TTC)

        TTCtext = "N/A";

    else

        TTCtext = sprintf("%.1f sec",TTC);

    end

    dashboard.String = sprintf( ...
        ['Ego Speed: %.0f km/h\n' ...
         'Distance: %.1f m     TTC: %s\n' ...
         'Risk: %s     Action: %s'], ...
        currentEgoSpeed*3.6, ...
        longitudinalDistance, ...
        TTCtext, ...
        risk, ...
        state);

    %% =====================================================
    % CAMERA FOLLOW
    %% =====================================================

    if egoX > 25

        xlim(ax,[egoX-25 egoX+40]);

    end

    drawnow;

    pause(0.025);

    %% End simulation

    if egoX >= 125
        break;
    end

end

dashboard.String = sprintf( ...
    ['DYNAMIC COLLISION AVOIDANCE COMPLETE\n' ...
     'MOVING VEHICLE DETECTED\n' ...
     'SAFE OVERTAKE COMPLETED']);

disp("-------------------------------------");
disp("PHASE 4 COMPLETED");
disp("Dynamic obstacle tracked");
disp("TTC calculated");
disp("Safe overtaking performed");
disp("-------------------------------------");


%% =========================================================
% FUNCTIONS
%% =========================================================

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
    'EdgeColor',[0.08 0.08 0.08]);

end


function vertices = ...
    cuboidVertices(center,L,W,H,yaw)

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

vertices(:,1) = vertices(:,1)+x;
vertices(:,2) = vertices(:,2)+y;
vertices(:,3) = vertices(:,3)+z;

end