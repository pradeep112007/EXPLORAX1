clear;
clc;
close all;

%% =========================================================
% FINAL INTEGRATED PROTOTYPE - VERSION 2
%
% Adaptive Path Planning and Collision Avoidance
% for Autonomous Vehicles on Unstructured Indian Roads
%
% LIVE CAMERA
%      ↓
% YOLO OBJECT DETECTION
%      ↓
% LEFT / CENTER / RIGHT FREE SPACE
%      ↓
% RISK ANALYSIS
%      ↓
% STRAIGHT / SLOW / LEFT / RIGHT / BRAKE
%      ↓
% MOVING 3D VEHICLE
%
% SOFTWARE / SIMULATION PROTOTYPE ONLY
%% =========================================================

disp("==============================================");
disp(" ADAPTIVE PATH PLANNING SYSTEM STARTING");
disp("==============================================");

%% =========================================================
% LOAD YOLO
%% =========================================================

disp("Loading YOLO v4...");

detector = yolov4ObjectDetector("tiny-yolov4-coco");

disp("YOLO loaded successfully.");

%% =========================================================
% CONNECT CAMERA
%% =========================================================

cam = webcam(1);

frame = snapshot(cam);

[frameH,frameW,~] = size(frame);

%% =========================================================
% DEFINE CAMERA NAVIGATION ZONES
%% =========================================================

zoneTop = round(frameH * 0.28);

zoneBottom = round(frameH * 0.96);

zoneHeight = zoneBottom - zoneTop;

third = round(frameW / 3);

leftZone = [
    1 ...
    zoneTop ...
    third ...
    zoneHeight
];

centerZone = [
    third+1 ...
    zoneTop ...
    third ...
    zoneHeight
];

rightZone = [
    2*third+1 ...
    zoneTop ...
    frameW-(2*third) ...
    zoneHeight
];

%% =========================================================
% CREATE WINDOW
%% =========================================================

fig = figure( ...
    'Name','Adaptive Path Planning - Final Prototype', ...
    'NumberTitle','off', ...
    'Color',[0.95 0.95 0.95]);

layout = tiledlayout( ...
    fig,1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% =========================================================
% LEFT SIDE - LIVE CAMERA
%% =========================================================

axCam = nexttile(layout,1);

camImage = imshow(frame,'Parent',axCam);

title(axCam, ...
    'LIVE AI PERCEPTION', ...
    'FontWeight','bold', ...
    'FontSize',14);

%% =========================================================
% RIGHT SIDE - 3D SIMULATION
%% =========================================================

ax3D = nexttile(layout,2);

hold(ax3D,'on');
grid(ax3D,'on');

axis(ax3D,'equal');

xlim(ax3D,[0 50]);
ylim(ax3D,[-10 10]);
zlim(ax3D,[0 8]);

xlabel(ax3D,'Forward Distance');
ylabel(ax3D,'Road Width');
zlabel(ax3D,'Height');

view(ax3D,40,24);

%% =========================================================
% GROUND
%% =========================================================

patch(ax3D, ...
    [0 50 50 0], ...
    [-10 -10 10 10], ...
    [0 0 0 0], ...
    [0.28 0.55 0.24], ...
    'EdgeColor','none');

%% =========================================================
% ROAD
%% =========================================================

patch(ax3D, ...
    [0 50 50 0], ...
    [-6 -6 6 6], ...
    [0.05 0.05 0.05 0.05], ...
    [0.20 0.20 0.20], ...
    'EdgeColor','none');

%% Road boundaries

plot3(ax3D, ...
    [0 50], ...
    [-6 -6], ...
    [0.1 0.1], ...
    'w', ...
    'LineWidth',2);

plot3(ax3D, ...
    [0 50], ...
    [6 6], ...
    [0.1 0.1], ...
    'w', ...
    'LineWidth',2);

%% =========================================================
% EGO VEHICLE SETTINGS
%% =========================================================

egoX = 6;
egoY = 0;

carLength = 4.5;
carWidth = 2;
carHeight = 1.3;

%% =========================================================
% CREATE EGO VEHICLE
%% =========================================================

egoBody = createCuboid( ...
    ax3D, ...
    [egoX egoY 0.65], ...
    carLength, ...
    carWidth, ...
    carHeight, ...
    0, ...
    [0.05 0.35 0.95]);

egoRoof = createCuboid( ...
    ax3D, ...
    [egoX egoY 1.55], ...
    2.5, ...
    1.7, ...
    0.7, ...
    0, ...
    [0.05 0.15 0.40]);

egoLabel = text( ...
    ax3D, ...
    egoX, ...
    egoY, ...
    3, ...
    'EGO VEHICLE', ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center');

%% =========================================================
% SAFE PATH
%% =========================================================

safePath = plot3( ...
    ax3D, ...
    nan,nan,nan, ...
    'LineWidth',5);

%% =========================================================
% MANEUVER MEMORY
%
% This fixes the LEFT / RIGHT issue.
%
% Once LEFT or RIGHT starts,
% continue it for several frames.
%% =========================================================

avoidanceMode = "NONE";

avoidanceCounter = 0;

avoidanceDuration = 55;

%% =========================================================
% MAIN LOOP
%% =========================================================

while ishandle(fig)

    %% =====================================================
    % LIVE CAMERA
    %% =====================================================

    frame = snapshot(cam);

    %% =====================================================
    % YOLO DETECTION
    %% =====================================================

    [bboxes,scores,labels] = detect( ...
        detector, ...
        frame, ...
        Threshold=0.35);

    labels = string(labels);

    %% =====================================================
    % KEEP ROAD OBJECTS ONLY
    %% =====================================================

    roadClasses = [
        "person"
        "car"
        "motorcycle"
        "bicycle"
        "bus"
        "truck"
    ];

    keep = ismember(labels,roadClasses);

    bboxes = bboxes(keep,:);

    scores = scores(keep);

    labels = labels(keep);

    %% =====================================================
    % RESET FRAME ANALYSIS
    %% =====================================================

    leftOcc = 0;
    centerOcc = 0;
    rightOcc = 0;

    highThreat = false;
    mediumThreat = false;

    centerVehicleThreat = false;

    centerVulnerableThreat = false;

    %% =====================================================
    % ANALYSE DETECTED OBJECTS
    %% =====================================================

    for i = 1:size(bboxes,1)

        box = bboxes(i,:);

        x = box(1);
        y = box(2);
        w = box(3);
        h = box(4);

        objectCenterX = x + w/2;

        objectBottom = y + h;

        %% -------------------------------------------------
        % OBJECT SIZE
        %% -------------------------------------------------

        areaRatio = ...
            (w*h) / ...
            (frameW*frameH);

        bottomRatio = ...
            objectBottom/frameH;

        %% -------------------------------------------------
        % LEFT / CENTER / RIGHT OCCUPANCY
        %% -------------------------------------------------

        if objectCenterX < frameW/3

            leftOcc = leftOcc + areaRatio;

        elseif objectCenterX < 2*frameW/3

            centerOcc = centerOcc + areaRatio;

        else

            rightOcc = rightOcc + areaRatio;

        end

        %% -------------------------------------------------
        % APPROXIMATE OBJECT PROXIMITY
        %% -------------------------------------------------

        closeObject = ...
            areaRatio > 0.055 || ...
            (bottomRatio > 0.82 && ...
            areaRatio > 0.020);

        mediumObject = ...
            areaRatio > 0.012 && ...
            bottomRatio > 0.45;

        %% -------------------------------------------------
        % OBJECT IN FORWARD CENTER
        %% -------------------------------------------------

        objectInCenter = ...
            objectCenterX > frameW/3 && ...
            objectCenterX < 2*frameW/3;

        %% -------------------------------------------------
        % THREAT CLASSIFICATION
        %% -------------------------------------------------

        if objectInCenter

            %% Person / bike / motorcycle

            vulnerable = ...
                labels(i) == "person" || ...
                labels(i) == "bicycle" || ...
                labels(i) == "motorcycle";

            %% Car / bus / truck

            vehicleObstacle = ...
                labels(i) == "car" || ...
                labels(i) == "bus" || ...
                labels(i) == "truck";

            if closeObject

                highThreat = true;

                if vulnerable

                    centerVulnerableThreat = true;

                end

                if vehicleObstacle

                    centerVehicleThreat = true;

                end

            elseif mediumObject

                mediumThreat = true;

                if vulnerable

                    centerVulnerableThreat = true;

                end

                if vehicleObstacle

                    centerVehicleThreat = true;

                end

            end

        end

    end

    %% =====================================================
    % FREE SPACE SCORES
    %% =====================================================

    leftFree = max(0,1-leftOcc);

    centerFree = max(0,1-centerOcc);

    rightFree = max(0,1-rightOcc);

    %% =====================================================
    % DECISION SYSTEM
    %% =====================================================

    risk = "SAFE";

    decision = "GO STRAIGHT";

    %% =====================================================
    % PRIORITY 1:
    % Vulnerable road user
    %% =====================================================

    if centerVulnerableThreat

        %% Cancel any active overtaking

        avoidanceMode = "NONE";

        avoidanceCounter = 0;

        if highThreat

            risk = "HIGH";

            decision = "BRAKE";

        else

            risk = "MEDIUM";

            decision = "SLOW";

        end

    %% =====================================================
    % PRIORITY 2:
    % Continue existing avoidance maneuver
    %% =====================================================

    elseif avoidanceMode ~= "NONE"

        risk = "HIGH";

        decision = avoidanceMode;

        avoidanceCounter = ...
            avoidanceCounter - 1;

        if avoidanceCounter <= 0

            avoidanceMode = "NONE";

        end

    %% =====================================================
    % PRIORITY 3:
    % Vehicle obstacle in center
    %% =====================================================

    elseif centerVehicleThreat

        if highThreat

            risk = "HIGH";

        else

            risk = "MEDIUM";

        end

        %% ---------------------------------------------
        % Compare left and right side
        %% ---------------------------------------------

        if leftFree > rightFree + 0.02

            decision = "MOVE LEFT";

        elseif rightFree > leftFree + 0.02

            decision = "MOVE RIGHT";

        else

            %% If both appear equally free,
            % choose LEFT.

            decision = "MOVE LEFT";

        end

        %% Lock maneuver

        avoidanceMode = decision;

        avoidanceCounter = ...
            avoidanceDuration;

    %% =====================================================
    % OTHER MEDIUM RISK
    %% =====================================================

    elseif mediumThreat

        risk = "MEDIUM";

        decision = "SLOW";

    %% =====================================================
    % SAFE
    %% =====================================================

    else

        risk = "SAFE";

        decision = "GO STRAIGHT";

    end

    %% =====================================================
    % VEHICLE CONTROL
    %% =====================================================

    switch decision

        %% -------------------------------------------------
        % GO STRAIGHT
        %% -------------------------------------------------

        case "GO STRAIGHT"

            targetY = 0;

            forwardStep = 0.18;

            simulatedSpeed = 30;

        %% -------------------------------------------------
        % SLOW
        %% -------------------------------------------------

        case "SLOW"

            targetY = 0;

            forwardStep = 0.06;

            simulatedSpeed = 10;

        %% -------------------------------------------------
        % MOVE LEFT
        %% -------------------------------------------------

        case "MOVE LEFT"

            targetY = 3.5;

            forwardStep = 0.14;

            simulatedSpeed = 24;

        %% -------------------------------------------------
        % MOVE RIGHT
        %% -------------------------------------------------

        case "MOVE RIGHT"

            targetY = -3.5;

            forwardStep = 0.14;

            simulatedSpeed = 24;

        %% -------------------------------------------------
        % BRAKE
        %% -------------------------------------------------

        case "BRAKE"

            targetY = egoY;

            forwardStep = 0;

            simulatedSpeed = 0;

    end

    %% =====================================================
    % SMOOTH LATERAL STEERING
    %% =====================================================

    lateralError = ...
        targetY - egoY;

    maxSideStep = 0.10;

    lateralStep = max( ...
        min(lateralError,maxSideStep), ...
        -maxSideStep);

    %% Change road position

    egoY = egoY + lateralStep;

    %% Move forward

    egoX = egoX + forwardStep;

    %% =====================================================
    % CALCULATE CAR ROTATION
    %% =====================================================

    if forwardStep > 0

        yaw = atan2( ...
            lateralStep, ...
            forwardStep);

    else

        yaw = 0;

    end

    %% =====================================================
    % LOOP ROAD
    %% =====================================================

    if egoX > 43

        egoX = 6;

        egoY = 0;

        avoidanceMode = "NONE";

        avoidanceCounter = 0;

    end

    %% =====================================================
    % UPDATE CAR BODY
    %% =====================================================

    set( ...
        egoBody, ...
        'Vertices', ...
        cuboidVertices( ...
            [egoX egoY 0.65], ...
            carLength, ...
            carWidth, ...
            carHeight, ...
            yaw));

    %% =====================================================
    % UPDATE CAR ROOF
    %% =====================================================

    set( ...
        egoRoof, ...
        'Vertices', ...
        cuboidVertices( ...
            [egoX egoY 1.55], ...
            2.5, ...
            1.7, ...
            0.7, ...
            yaw));

    %% =====================================================
    % UPDATE CAR LABEL
    %% =====================================================

    egoLabel.Position = ...
        [egoX egoY 3];

    %% =====================================================
    % CREATE ADAPTIVE PATH
    %% =====================================================

   %% =====================================================
% CREATE VISIBLE ADAPTIVE PATH
%% =====================================================

% Path starts slightly in front of ego vehicle
pathStartX = egoX + 2;
pathEndX   = min(48, egoX + 28);

pathX = linspace(pathStartX,pathEndX,100);
pathY = egoY * ones(size(pathX));

switch decision

    %% =================================================
    % MOVE LEFT
    %% =================================================
    case "MOVE LEFT"

        avoidY = 4.2;

        for j = 1:length(pathX)

            t = (j-1)/(length(pathX)-1);

            if t < 0.40

                % Move away from obstacle
                localT = t/0.40;

                pathY(j) = ...
                    egoY + ...
                    (avoidY-egoY) * smoothStep(localT);

            elseif t < 0.70

                % Stay on avoidance side
                pathY(j) = avoidY;

            else

                % Return toward center
                localT = (t-0.70)/0.30;

                pathY(j) = ...
                    avoidY * ...
                    (1-smoothStep(localT));

            end

        end

    %% =================================================
    % MOVE RIGHT
    %% =================================================
    case "MOVE RIGHT"

        avoidY = -4.2;

        for j = 1:length(pathX)

            t = (j-1)/(length(pathX)-1);

            if t < 0.40

                % Move away from obstacle
                localT = t/0.40;

                pathY(j) = ...
                    egoY + ...
                    (avoidY-egoY) * smoothStep(localT);

            elseif t < 0.70

                % Stay on avoidance side
                pathY(j) = avoidY;

            else

                % Return toward center
                localT = (t-0.70)/0.30;

                pathY(j) = ...
                    avoidY * ...
                    (1-smoothStep(localT));

            end

        end

    %% =================================================
    % BRAKE
    %% =================================================
    case "BRAKE"

        pathEndX = min(48,egoX + 6);

        pathX = linspace( ...
            egoX+2, ...
            pathEndX, ...
            30);

        pathY = ...
            egoY * ones(size(pathX));

    %% =================================================
    % SLOW
    %% =================================================
    case "SLOW"

        pathEndX = min(48,egoX + 15);

        pathX = linspace( ...
            egoX+2, ...
            pathEndX, ...
            50);

        pathY = ...
            egoY * ones(size(pathX));

    %% =================================================
    % GO STRAIGHT
    %% =================================================
    otherwise

        pathY = ...
            egoY * ones(size(pathX));

end

%% UPDATE PATH

set(safePath, ...
    'XData',pathX, ...
    'YData',pathY, ...
    'ZData',0.18*ones(size(pathX)));
        %% ---------------------------------------------
        % SMOOTH PATH
        %% ---------------------------------------------

        for j = 1:length(pathX)

            t = ...
                (j-1) / ...
                (length(pathX)-1);

            pathY(j) = ...
                egoY + ...
                (pathTargetY-egoY) * ...
                smoothStep(t);

        end

    end

    %% =====================================================
    % UPDATE PATH
    %% =====================================================

    set( ...
        safePath, ...
        'XData',pathX, ...
        'YData',pathY, ...
        'ZData', ...
        0.15*ones(size(pathX)));

    %% =====================================================
    % CREATE YOLO LABEL TEXT
    %% =====================================================

    detectionText = ...
        strings(length(labels),1);

    for i = 1:length(labels)

        detectionText(i) = ...
            upper(labels(i)) + ...
            " " + ...
            string( ...
            round(scores(i)*100)) + ...
            "%";

    end

    %% =====================================================
    % DRAW YOLO BOUNDING BOXES
    %% =====================================================

    if ~isempty(bboxes)

        output = ...
            insertObjectAnnotation( ...
            frame, ...
            "rectangle", ...
            bboxes, ...
            cellstr(detectionText), ...
            'LineWidth',4);

    else

        output = frame;

    end

    %% =====================================================
    % DRAW NAVIGATION ZONES
    %% =====================================================

    output = insertShape( ...
        output, ...
        "Rectangle", ...
        leftZone, ...
        'LineWidth',3);

    output = insertShape( ...
        output, ...
        "Rectangle", ...
        centerZone, ...
        'LineWidth',4);

    output = insertShape( ...
        output, ...
        "Rectangle", ...
        rightZone, ...
        'LineWidth',3);

    %% =====================================================
    % LEFT ZONE INFORMATION
    %% =====================================================

    output = insertText( ...
        output, ...
        [15 zoneTop+10], ...
        sprintf( ...
        "LEFT\nFREE %.0f%%", ...
        leftFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.7);

    %% =====================================================
    % CENTER ZONE INFORMATION
    %% =====================================================

    output = insertText( ...
        output, ...
        [third+15 zoneTop+10], ...
        sprintf( ...
        "CENTER\nFREE %.0f%%", ...
        centerFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.7);

    %% =====================================================
    % RIGHT ZONE INFORMATION
    %% =====================================================

    output = insertText( ...
        output, ...
        [2*third+15 zoneTop+10], ...
        sprintf( ...
        "RIGHT\nFREE %.0f%%", ...
        rightFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.7);

    %% =====================================================
    % STATUS DISPLAY
    %% =====================================================

    statusMessage = ...
        "RISK: " + risk + ...
        " | DECISION: " + decision + ...
        " | SPEED: " + ...
        string(simulatedSpeed) + ...
        " km/h";

    output = insertText( ...
        output, ...
        [20 frameH-80], ...
        statusMessage, ...
        'FontSize',20, ...
        'BoxOpacity',0.85);

    %% =====================================================
    % CAMERA UPDATE
    %% =====================================================

    set( ...
        camImage, ...
        'CData', ...
        output);

    %% =====================================================
    % CAMERA TITLE
    %% =====================================================

    title( ...
        axCam, ...
        sprintf( ...
        ['LIVE YOLO PERCEPTION\n' ...
        'Detected Objects: %d'], ...
        length(labels)), ...
        'FontWeight','bold');

    %% =====================================================
    % 3D TITLE
    %% =====================================================

    title( ...
        ax3D, ...
        sprintf( ...
        ['3D ADAPTIVE VEHICLE\n' ...
        'Decision: %s | Speed: %d km/h\n' ...
        'Vehicle Y: %.2f'], ...
        decision, ...
        simulatedSpeed, ...
        egoY), ...
        'FontWeight','bold');

    %% =====================================================
    % MAIN TITLE
    %% =====================================================

    title( ...
        layout, ...
        sprintf( ...
        ['ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE\n' ...
        'Risk: %s     Decision: %s'], ...
        risk, ...
        decision), ...
        'FontSize',15, ...
        'FontWeight','bold');

    drawnow limitrate;



%% =========================================================
% RELEASE CAMERA
%% =========================================================

clear cam;

disp("==============================================");
disp(" SYSTEM STOPPED");
disp(" CAMERA RELEASED");
disp("==============================================");


%% =========================================================
% SMOOTH STEP FUNCTION
%% =========================================================

function output = smoothStep(t)

output = ...
    t.^2 .* ...
    (3 - 2*t);

end


%% =========================================================
% CREATE CUBOID
%% =========================================================

function h = createCuboid( ...
    ax,center,L,W,H,yaw,color)

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

h = patch( ...
    ax, ...
    'Vertices',vertices, ...
    'Faces',faces, ...
    'FaceColor',color, ...
    'EdgeColor',[0.1 0.1 0.1]);

end


%% =========================================================
% GENERATE CUBOID VERTICES
%% =========================================================

function vertices = cuboidVertices( ...
    center,L,W,H,yaw)

x = center(1);
y = center(2);
z = center(3);

%% Original cuboid around origin

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

%% Rotation matrix

R = [

    cos(yaw) -sin(yaw) 0

    sin(yaw)  cos(yaw) 0

    0         0        1

];

%% Rotate

vertices = ...
    localVertices * R';

%% Move to vehicle position

vertices(:,1) = ...
    vertices(:,1) + x;

vertices(:,2) = ...
    vertices(:,2) + y;

vertices(:,3) = ...
    vertices(:,3) + z;

end