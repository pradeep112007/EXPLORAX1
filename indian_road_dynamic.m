clear;
clc;
close all;

%% =========================================================
% PHASE 5
% INDIAN ROAD DYNAMIC SCENARIO
% Pedestrian + Bike + TTC + Avoid / Slow / Brake
%% =========================================================

%% SETTINGS

roadLength = 140;
roadWidth  = 12;
dt = 0.05;

% Ego car
egoX = 5;
egoY = 0;
egoSpeed = 8.33;      % 30 km/h

% Bike
bikeX = 38;
bikeY = -2.5;
bikeSpeed = 4.5;      % m/s

% Pedestrian
pedX = 58;
pedY = 7;
pedSpeed = 1.8;       % crossing road downward

% Safety thresholds
detectionRange = 30;
highRiskDistance = 10;
emergencyDistance = 6;

%% =========================================================
% CREATE FIGURE
%% =========================================================

fig = figure( ...
    'Name','Indian Road Adaptive Collision Avoidance', ...
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

view(ax,40,23);

%% =========================================================
% GROUND
%% =========================================================

patch(ax, ...
    [0 roadLength roadLength 0], ...
    [-15 -15 15 15], ...
    [0 0 0 0], ...
    [0.28 0.55 0.24], ...
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

plot3(ax,[0 roadLength],[-6 -6],[0.08 0.08], ...
    'w','LineWidth',2);

plot3(ax,[0 roadLength],[6 6],[0.08 0.08], ...
    'w','LineWidth',2);

%% =========================================================
% ROAD SIDE OBJECTS
%% =========================================================

for x = 10:15:130

    createCuboid(ax,[x -10 1], ...
        1.3,1.3,2,0,[0.4 0.25 0.1]);

    createCuboid(ax,[x+5 10 1], ...
        1.3,1.3,2,0,[0.4 0.25 0.1]);

end

%% =========================================================
% CREATE EGO CAR
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
% CREATE BIKE
%% =========================================================

bikeBody = createCuboid(ax, ...
    [bikeX bikeY 0.45], ...
    2,0.7,0.9,0, ...
    [0.95 0.55 0.05]);

%% =========================================================
% CREATE PEDESTRIAN
%% =========================================================

pedBody = createCuboid(ax, ...
    [pedX pedY 0.9], ...
    0.6,0.6,1.8,0, ...
    [0.15 0.8 0.2]);

%% =========================================================
% LABELS
%% =========================================================

bikeLabel = text(ax,bikeX,bikeY,2.3, ...
    'BIKE', ...
    'FontWeight','bold');

pedLabel = text(ax,pedX,pedY,2.5, ...
    'PEDESTRIAN', ...
    'FontWeight','bold');

%% =========================================================
% DASHBOARD
%% =========================================================

dashboard = annotation(fig, ...
    'textbox', ...
    [0.18 0.81 0.65 0.16], ...
    'String','SYSTEM STARTING', ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center', ...
    'BackgroundColor',[1 1 1], ...
    'EdgeColor',[0.2 0.2 0.2]);

%% =========================================================
% PATH HISTORY
%% =========================================================

pathX = [];
pathY = [];

pathPlot = plot3(ax,nan,nan,nan, ...
    'LineWidth',2.5);

%% =========================================================
% SYSTEM VARIABLES
%% =========================================================

targetY = 0;

state = "CRUISE";

currentSpeed = egoSpeed;

pedestrianCleared = false;

%% =========================================================
% SIMULATION
%% =========================================================

for frame = 1:1200

    %% -----------------------------------------------------
    % MOVE BIKE
    %% -----------------------------------------------------

    bikeX = bikeX + bikeSpeed*dt;

    %% -----------------------------------------------------
    % MOVE PEDESTRIAN ACROSS ROAD
    %% -----------------------------------------------------

    if pedY > -7

        pedY = pedY - pedSpeed*dt;

    else

        pedestrianCleared = true;

    end

    %% -----------------------------------------------------
    % DISTANCE TO BIKE
    %% -----------------------------------------------------

    bikeDistance = sqrt( ...
        (bikeX-egoX)^2 + ...
        (bikeY-egoY)^2);

    %% -----------------------------------------------------
    % DISTANCE TO PEDESTRIAN
    %% -----------------------------------------------------

    pedDistance = sqrt( ...
        (pedX-egoX)^2 + ...
        (pedY-egoY)^2);

    %% -----------------------------------------------------
    % TTC WITH BIKE
    %% -----------------------------------------------------

    relativeSpeedBike = currentSpeed-bikeSpeed;

    if bikeX > egoX && relativeSpeedBike > 0

        TTCbike = ...
            (bikeX-egoX) / relativeSpeedBike;

    else

        TTCbike = inf;

    end

    %% =====================================================
    % DECISION LOGIC
    %% =====================================================

    % Pedestrian gets highest priority

    if pedX > egoX && ...
       pedDistance < emergencyDistance && ...
       abs(pedY) < 4

        state = "EMERGENCY BRAKE";

        targetY = egoY;

        currentSpeed = ...
            max(0,currentSpeed-0.5);

    elseif pedX > egoX && ...
           pedDistance < highRiskDistance && ...
           abs(pedY) < 5

        state = "SLOW FOR PEDESTRIAN";

        targetY = 0;

        currentSpeed = ...
            max(3,currentSpeed-0.12);

    elseif pedX > egoX && ...
           pedDistance < detectionRange && ...
           abs(pedY) < 6

        state = "PEDESTRIAN DETECTED";

        currentSpeed = ...
            max(5,currentSpeed-0.05);

    elseif TTCbike < 4

        state = "AVOID BIKE";

        targetY = 3.3;

        currentSpeed = egoSpeed;

    elseif pedestrianCleared

        state = "CRUISE";

        targetY = 0;

        currentSpeed = egoSpeed;

    else

        state = "CRUISE";

        targetY = 0;

        currentSpeed = egoSpeed;

    end

    %% =====================================================
    % RISK LEVEL
    %% =====================================================

    if state == "EMERGENCY BRAKE"

        risk = "CRITICAL";

    elseif state == "SLOW FOR PEDESTRIAN" || ...
           state == "AVOID BIKE"

        risk = "HIGH";

    elseif state == "PEDESTRIAN DETECTED"

        risk = "MEDIUM";

    else

        risk = "SAFE";

    end

    %% =====================================================
    % LATERAL STEERING
    %% =====================================================

    lateralError = targetY - egoY;

    maxLateralStep = 0.07;

    lateralStep = max( ...
        min(lateralError,maxLateralStep), ...
        -maxLateralStep);

    egoY = egoY + lateralStep;

    %% =====================================================
    % MOVE EGO FORWARD
    %% =====================================================

    egoX = egoX + currentSpeed*dt;

    %% =====================================================
    % VEHICLE YAW
    %% =====================================================

    if currentSpeed > 0

        yaw = atan2( ...
            lateralStep, ...
            currentSpeed*dt);

    else

        yaw = 0;

    end

    %% =====================================================
    % UPDATE EGO
    %% =====================================================

    set(egoBody, ...
        'Vertices', ...
        cuboidVertices( ...
        [egoX egoY 0.65], ...
        4.5,2,1.3,yaw));

    set(egoRoof, ...
        'Vertices', ...
        cuboidVertices( ...
        [egoX egoY 1.55], ...
        2.5,1.7,0.7,yaw));

    %% =====================================================
    % UPDATE BIKE
    %% =====================================================

    set(bikeBody, ...
        'Vertices', ...
        cuboidVertices( ...
        [bikeX bikeY 0.45], ...
        2,0.7,0.9,0));

    bikeLabel.Position = ...
        [bikeX bikeY 2.3];

    %% =====================================================
    % UPDATE PEDESTRIAN
    %% =====================================================

    set(pedBody, ...
        'Vertices', ...
        cuboidVertices( ...
        [pedX pedY 0.9], ...
        0.6,0.6,1.8,0));

    pedLabel.Position = ...
        [pedX pedY 2.5];

    %% =====================================================
    % PATH HISTORY
    %% =====================================================

    pathX(end+1) = egoX;
    pathY(end+1) = egoY;

    set(pathPlot, ...
        'XData',pathX, ...
        'YData',pathY, ...
        'ZData',0.12*ones(size(pathX)));

    %% =====================================================
    % TTC TEXT
    %% =====================================================

    if isinf(TTCbike)

        bikeTTCtext = "N/A";

    else

        bikeTTCtext = ...
            sprintf("%.1f sec",TTCbike);

    end

    %% =====================================================
    % DASHBOARD
    %% =====================================================

    dashboard.String = sprintf( ...
        ['INDIAN ROAD COLLISION AVOIDANCE\n' ...
         'Speed: %.1f km/h\n' ...
         'Bike Distance: %.1f m | TTC: %s\n' ...
         'Pedestrian Distance: %.1f m\n' ...
         'Risk: %s | Decision: %s'], ...
        currentSpeed*3.6, ...
        bikeDistance, ...
        bikeTTCtext, ...
        pedDistance, ...
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

    %% =====================================================
    % END CONDITION
    %% =====================================================

    if egoX >= 125
        break;
    end

end

dashboard.String = sprintf( ...
    ['SIMULATION COMPLETE\n' ...
     'DYNAMIC OBJECTS HANDLED SUCCESSFULLY']);

disp("---------------------------------------");
disp("PHASE 5 COMPLETE");
disp("Bike tracked");
disp("Pedestrian monitored");
disp("Risk calculated");
disp("Adaptive decision performed");
disp("---------------------------------------");


%% =========================================================
% FUNCTIONS
%% =========================================================

function h = createCuboid(ax,center,L,W,H,yaw,color)

vertices = cuboidVertices( ...
    center,L,W,H,yaw);

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