clear;
clc;
close all;

%% =========================================================
% FINAL DEMO V3
%
% Adaptive Path Planning and Collision Avoidance
% for Autonomous Vehicles on Unstructured Indian Roads
%
% LIVE CAMERA
%      ↓
% YOLO OBJECT DETECTION
%      ↓
% LEFT / CENTER / RIGHT ANALYSIS
%      ↓
% RISK ASSESSMENT
%      ↓
% LEFT / RIGHT / SLOW / BRAKE
%      ↓
% MOVING 3D VEHICLE + ADAPTIVE PATH
%
% SOFTWARE / SIMULATION PROTOTYPE ONLY
%% =========================================================

disp("==============================================");
disp(" ADAPTIVE PATH PLANNING SYSTEM");
disp(" STARTING...");
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
% CAMERA NAVIGATION ZONES
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
    third + 1 ...
    zoneTop ...
    third ...
    zoneHeight
];

rightZone = [
    2*third + 1 ...
    zoneTop ...
    frameW - 2*third ...
    zoneHeight
];

%% =========================================================
% CREATE MAIN WINDOW
%% =========================================================

fig = figure( ...
    'Name','Adaptive Path Planning - Final Prototype', ...
    'NumberTitle','off', ...
    'Color',[0.95 0.95 0.95]);

layout = tiledlayout( ...
    fig, ...
    1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% =========================================================
% LEFT PANEL - LIVE CAMERA
%% =========================================================

axCam = nexttile(layout,1);

camImage = imshow(frame,'Parent',axCam);

title( ...
    axCam, ...
    'LIVE AI PERCEPTION', ...
    'FontWeight','bold', ...
    'FontSize',14);

%% =========================================================
% RIGHT PANEL - 3D SIMULATION
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

patch( ...
    ax3D, ...
    [0 50 50 0], ...
    [-10 -10 10 10], ...
    [0 0 0 0], ...
    [0.28 0.55 0.24], ...
    'EdgeColor','none');

%% =========================================================
% ROAD
%% =========================================================

patch( ...
    ax3D, ...
    [0 50 50 0], ...
    [-6 -6 6 6], ...
    [0.05 0.05 0.05 0.05], ...
    [0.20 0.20 0.20], ...
    'EdgeColor','none');

%% Road boundaries

plot3( ...
    ax3D, ...
    [0 50], ...
    [-6 -6], ...
    [0.1 0.1], ...
    'w', ...
    'LineWidth',2);

plot3( ...
    ax3D, ...
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
% CREATE EGO CAR
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
    'HorizontalAlignment','center', ...
    'FontWeight','bold');

%% =========================================================
% PLANNED PATH
%% =========================================================

safePath = plot3( ...
    ax3D, ...
    nan,nan,nan, ...
    'LineWidth',5);

%% =========================================================
% AVOIDANCE MEMORY
%
% Prevents LEFT / RIGHT from disappearing immediately.
%% =========================================================

avoidanceMode = "NONE";

avoidanceCounter = 0;

avoidanceDuration = 35;

%% =========================================================
% MAIN LOOP
%% =========================================================

while ishandle(fig)

    %% =====================================================
    % GET LIVE FRAME
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
    % KEEP ROAD-RELATED OBJECTS
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
    % RESET FRAME VARIABLES
    %% =====================================================

    leftOcc = 0;
    centerOcc = 0;
    rightOcc = 0;

    highThreat = false;
    mediumThreat = false;

    centerVehicleThreat = false;
    centerVulnerableThreat = false;

    %% =====================================================
    % ANALYSE DETECTIONS
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
        % OBJECT SIZE RELATIVE TO IMAGE
        %% -------------------------------------------------

        areaRatio = ...
            (w*h) / ...
            (frameW*frameH);

        bottomRatio = ...
            objectBottom / frameH;

        %% -------------------------------------------------
        % WHICH SIDE?
        %% -------------------------------------------------

        if objectCenterX < frameW/3

            leftOcc = ...
                leftOcc + areaRatio;

        elseif objectCenterX < 2*frameW/3

            centerOcc = ...
                centerOcc + areaRatio;

        else

            rightOcc = ...
                rightOcc + areaRatio;

        end

        %% -------------------------------------------------
        % APPROXIMATE CLOSENESS
        %
        % Larger object + lower in frame
        % approximately means closer.
        %% -------------------------------------------------

        closeObject = ...
            areaRatio > 0.055 || ...
            (bottomRatio > 0.82 && ...
             areaRatio > 0.020);

        mediumObject = ...
            areaRatio > 0.012 && ...
            bottomRatio > 0.45;

        %% -------------------------------------------------
        % OBJECT IN FORWARD CENTER AREA
        %% -------------------------------------------------

        objectInCenter = ...
            objectCenterX > frameW/3 && ...
            objectCenterX < 2*frameW/3;

        %% -------------------------------------------------
        % CLASS TYPE
        %% -------------------------------------------------

        vulnerable = ...
            labels(i) == "person" || ...
            labels(i) == "bicycle" || ...
            labels(i) == "motorcycle";

        vehicleObstacle = ...
            labels(i) == "car" || ...
            labels(i) == "bus" || ...
            labels(i) == "truck";

        %% -------------------------------------------------
        % THREAT ANALYSIS
        %% -------------------------------------------------

        if objectInCenter

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

    %% -----------------------------------------------------
    % PRIORITY 1
    % PERSON / BIKE / MOTORCYCLE
    %% -----------------------------------------------------

    if centerVulnerableThreat

        %% Cancel vehicle avoidance
        avoidanceMode = "NONE";
        avoidanceCounter = 0;

        if highThreat

            risk = "HIGH";
            decision = "BRAKE";

        else

            risk = "MEDIUM";
            decision = "SLOW";

        end

    %% -----------------------------------------------------
    % PRIORITY 2
    % CONTINUE ACTIVE LEFT / RIGHT MANEUVER
    %% -----------------------------------------------------

    elseif avoidanceMode ~= "NONE"

        risk = "HIGH";

        decision = avoidanceMode;

        avoidanceCounter = ...
            avoidanceCounter - 1;

        if avoidanceCounter <= 0

            avoidanceMode = "NONE";

        end

    %% -----------------------------------------------------
    % PRIORITY 3
    % CAR / BUS / TRUCK IN CENTER
    %% -----------------------------------------------------

    elseif centerVehicleThreat

        if highThreat

            risk = "HIGH";

        else

            risk = "MEDIUM";

        end

        %% -------------------------------------------------
        % CHOOSE FREER SIDE
        %% -------------------------------------------------

        if leftFree > rightFree + 0.02

            decision = "MOVE LEFT";

        elseif rightFree > leftFree + 0.02

            decision = "MOVE RIGHT";

        else

            %% Both roughly equal:
            % choose LEFT for demonstration.

            decision = "MOVE LEFT";

        end

        %% Lock decision

        avoidanceMode = decision;

        avoidanceCounter = ...
            avoidanceDuration;

    %% -----------------------------------------------------
    % OTHER MEDIUM THREAT
    %% -----------------------------------------------------

    elseif mediumThreat

        risk = "MEDIUM";

        decision = "SLOW";

    %% -----------------------------------------------------
    % SAFE
    %% -----------------------------------------------------

    else

        risk = "SAFE";

        decision = "GO STRAIGHT";

    end

    %% =====================================================
    % VEHICLE MOVEMENT
    %% =====================================================

    switch decision

        %% -------------------------------------------------
        % STRAIGHT
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
        % LEFT
        %% -------------------------------------------------

        case "MOVE LEFT"

            targetY = 4.2;

            forwardStep = 0.14;

            simulatedSpeed = 24;

        %% -------------------------------------------------
        % RIGHT
        %% -------------------------------------------------

        case "MOVE RIGHT"

            targetY = -4.2;

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
    % SMOOTH SIDEWAYS STEERING
    %% =====================================================

    lateralError = ...
        targetY - egoY;

    maxSideStep = 0.12;

    lateralStep = max( ...
        min(lateralError,maxSideStep), ...
        -maxSideStep);

    egoY = ...
        egoY + lateralStep;

    %% Forward movement

    egoX = ...
        egoX + forwardStep;

    %% =====================================================
    % CALCULATE VEHICLE ROTATION
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
    % UPDATE EGO BODY
    %% =====================================================

    bodyVertices = cuboidVertices( ...
        [egoX egoY 0.65], ...
        carLength, ...
        carWidth, ...
        carHeight, ...
        yaw);

    set( ...
        egoBody, ...
        'Vertices', ...
        bodyVertices);

    %% =====================================================
    % UPDATE EGO ROOF
    %% =====================================================

    roofVertices = cuboidVertices( ...
        [egoX egoY 1.55], ...
        2.5, ...
        1.7, ...
        0.7, ...
        yaw);

    set( ...
        egoRoof, ...
        'Vertices', ...
        roofVertices);

    %% Update label

    egoLabel.Position = ...
        [egoX egoY 3];

    %% =====================================================
    % CREATE VISIBLE ADAPTIVE PATH
    %% =====================================================

    pathStartX = egoX + 2;

    pathEndX = ...
        min(48,egoX + 28);

    pathX = linspace( ...
        pathStartX, ...
        pathEndX, ...
        100);

    pathY = ...
        egoY * ones(size(pathX));

    switch decision

        %% =================================================
        % LEFT PATH
        %% =================================================

        case "MOVE LEFT"

            avoidY = 4.2;

            for j = 1:length(pathX)

                t = ...
                    (j-1) / ...
                    (length(pathX)-1);

                if t < 0.35

                    localT = ...
                        t / 0.35;

                    pathY(j) = ...
                        egoY + ...
                        (avoidY-egoY) * ...
                        smoothStep(localT);

                elseif t < 0.70

                    pathY(j) = avoidY;

                else

                    localT = ...
                        (t-0.70) / 0.30;

                    pathY(j) = ...
                        avoidY * ...
                        (1-smoothStep(localT));

                end

            end

        %% =================================================
        % RIGHT PATH
        %% =================================================

        case "MOVE RIGHT"

            avoidY = -4.2;

            for j = 1:length(pathX)

                t = ...
                    (j-1) / ...
                    (length(pathX)-1);

                if t < 0.35

                    localT = ...
                        t / 0.35;

                    pathY(j) = ...
                        egoY + ...
                        (avoidY-egoY) * ...
                        smoothStep(localT);

                elseif t < 0.70

                    pathY(j) = avoidY;

                else

                    localT = ...
                        (t-0.70) / 0.30;

                    pathY(j) = ...
                        avoidY * ...
                        (1-smoothStep(localT));

                end

            end

        %% =================================================
        % BRAKE PATH
        %% =================================================

        case "BRAKE"

            pathEndX = ...
                min(48,egoX + 6);

            pathX = linspace( ...
                egoX+2, ...
                pathEndX, ...
                30);

            pathY = ...
                egoY * ones(size(pathX));

        %% =================================================
        % SLOW PATH
        %% =================================================

        case "SLOW"

            pathEndX = ...
                min(48,egoX + 15);

            pathX = linspace( ...
                egoX+2, ...
                pathEndX, ...
                50);

            %% Smoothly guide back toward center

            pathY = zeros(size(pathX));

            for j = 1:length(pathX)

                t = ...
                    (j-1) / ...
                    (length(pathX)-1);

                pathY(j) = ...
                    egoY * ...
                    (1-smoothStep(t));

            end

        %% =================================================
        % STRAIGHT
        %% =================================================

        otherwise

            %% If vehicle was previously left/right,
            % show path returning to center.

            pathY = zeros(size(pathX));

            for j = 1:length(pathX)

                t = ...
                    (j-1) / ...
                    (length(pathX)-1);

                pathY(j) = ...
                    egoY * ...
                    (1-smoothStep(t));

            end

    end

    %% =====================================================
    % UPDATE 3D PATH
    %% =====================================================

    set( ...
        safePath, ...
        'XData', ...
        pathX, ...
        'YData', ...
        pathY, ...
        'ZData', ...
        0.18*ones(size(pathX)));

    %% =====================================================
    % YOLO TEXT
    %% =====================================================

    detectionText = ...
        strings(length(labels),1);

    for i = 1:length(labels)

        detectionText(i) = ...
            upper(labels(i)) + ...
            " " + ...
            string(round(scores(i)*100)) + ...
            "%";

    end

    %% =====================================================
    % DRAW YOLO BOXES
    %% =====================================================

    if ~isempty(bboxes)

        output = insertObjectAnnotation( ...
            frame, ...
            "rectangle", ...
            bboxes, ...
            cellstr(detectionText), ...
            'LineWidth',4);

    else

        output = frame;

    end

    %% =====================================================
    % DRAW LEFT / CENTER / RIGHT ZONES
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
    % LEFT FREE SPACE
    %% =====================================================

    output = insertText( ...
        output, ...
        [15 zoneTop+10], ...
        sprintf( ...
        "LEFT\nFREE %.0f%%", ...
        leftFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.70);

    %% =====================================================
    % CENTER FREE SPACE
    %% =====================================================

    output = insertText( ...
        output, ...
        [third+15 zoneTop+10], ...
        sprintf( ...
        "CENTER\nFREE %.0f%%", ...
        centerFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.70);

    %% =====================================================
    % RIGHT FREE SPACE
    %% =====================================================

    output = insertText( ...
        output, ...
        [2*third+15 zoneTop+10], ...
        sprintf( ...
        "RIGHT\nFREE %.0f%%", ...
        rightFree*100), ...
        'FontSize',16, ...
        'BoxOpacity',0.70);

    %% =====================================================
    % CAMERA STATUS
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
    % UPDATE CAMERA
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
        'Lateral Position Y: %.2f'], ...
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

end

%% =========================================================
% RELEASE CAMERA
%% =========================================================

clear cam;

disp("==============================================");
disp(" SYSTEM STOPPED");
disp(" CAMERA RELEASED");
disp("==============================================");


%% =========================================================
% FUNCTION - SMOOTH STEP
%% =========================================================

function output = smoothStep(t)

output = ...
    t.^2 .* ...
    (3 - 2*t);

end


%% =========================================================
% FUNCTION - CREATE CUBOID
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
% FUNCTION - CUBOID VERTICES
%% =========================================================

function vertices = cuboidVertices( ...
    center,L,W,H,yaw)

x = center(1);
y = center(2);
z = center(3);

%% Cuboid around origin

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

%% Move to world position

vertices(:,1) = ...
    vertices(:,1) + x;

vertices(:,2) = ...
    vertices(:,2) + y;

vertices(:,3) = ...
    vertices(:,3) + z;

end