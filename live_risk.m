clear;
clc;
close all;

%% =========================================================
% PHASE 8
% LIVE CAMERA + YOLO + RISK ANALYSIS + 3D PATH DECISION
%
% IMPORTANT:
% This is a SOFTWARE/SIMULATION prototype.
% It does NOT control a real vehicle.
%% =========================================================

disp("Loading YOLO...");
detector = yolov4ObjectDetector("tiny-yolov4-coco");

%% Connect camera
cam = webcam(1);

%% Capture one frame to determine image size
frame = snapshot(cam);

[frameH,frameW,~] = size(frame);

%% =========================================================
% CREATE DASHBOARD
%% =========================================================

fig = figure( ...
    'Name','Adaptive Path Planning - Live AI Dashboard', ...
    'NumberTitle','off', ...
    'Color',[0.94 0.94 0.94]);

layout = tiledlayout(fig,1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% =========================================================
% LEFT SIDE - CAMERA
%% =========================================================

axCamera = nexttile(layout,1);

cameraImage = imshow(frame,'Parent',axCamera);

title(axCamera, ...
    'LIVE CAMERA + YOLO');

%% =========================================================
% RIGHT SIDE - 3D PATH VIEW
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

%% Ground

patch(ax3D, ...
    [0 50 50 0], ...
    [-10 -10 10 10], ...
    [0 0 0 0], ...
    [0.3 0.55 0.25], ...
    'EdgeColor','none');

%% Road

patch(ax3D, ...
    [0 50 50 0], ...
    [-6 -6 6 6], ...
    [0.05 0.05 0.05 0.05], ...
    [0.22 0.22 0.22], ...
    'EdgeColor','none');

%% Road boundaries

plot3(ax3D,[0 50],[-6 -6],[0.1 0.1], ...
    'LineWidth',2);

plot3(ax3D,[0 50],[6 6],[0.1 0.1], ...
    'LineWidth',2);

%% Ego vehicle

createCuboid(ax3D, ...
    [6 0 0.65], ...
    4.5,2,1.3, ...
    [0.1 0.35 0.95]);

createCuboid(ax3D, ...
    [6 0 1.55], ...
    2.4,1.7,0.7, ...
    [0.05 0.15 0.4]);

%% Planned path

plannedPath = plot3(ax3D, ...
    nan,nan,nan, ...
    'LineWidth',4);

%% =========================================================
% DANGER AREA IN CAMERA
%% =========================================================

% Central region represents the area directly ahead
dangerX = round(frameW*0.35);
dangerY = round(frameH*0.28);

dangerW = round(frameW*0.30);
dangerH = round(frameH*0.68);

dangerRect = ...
    [dangerX dangerY dangerW dangerH];

%% =========================================================
% MAIN LOOP
%% =========================================================

while ishandle(fig)

    %% Capture camera frame

    frame = snapshot(cam);

    %% -----------------------------------------------------
    % YOLO DETECTION
    %% -----------------------------------------------------

    [bboxes,scores,labels] = detect( ...
        detector, ...
        frame, ...
        Threshold=0.35);

    labelStrings = string(labels);

    %% Keep only road-related objects

    wantedObjects = [
        "person"
        "car"
        "motorcycle"
        "bus"
        "truck"
        "bicycle"
    ];

    keep = ismember(labelStrings,wantedObjects);

    bboxes = bboxes(keep,:);
    scores = scores(keep);
    labelStrings = labelStrings(keep);

    %% -----------------------------------------------------
    % VARIABLES
    %% -----------------------------------------------------

    risk = "SAFE";
    decision = "GO STRAIGHT";

    leftOccupancy = 0;
    rightOccupancy = 0;

    mediumThreat = false;
    highThreat = false;

    vulnerableRoadUser = false;

    %% =====================================================
    % ANALYSE DETECTED OBJECTS
    %% =====================================================

    for i = 1:size(bboxes,1)

        box = bboxes(i,:);

        x = box(1);
        y = box(2);
        w = box(3);
        h = box(4);

        objectCentreX = x + w/2;

        boxBottom = y + h;

        %% Object size relative to image

        areaRatio = ...
            (w*h)/(frameW*frameH);

        bottomRatio = ...
            boxBottom/frameH;

        %% -------------------------------------------------
        % CHECK LEFT / RIGHT OCCUPANCY
        %% -------------------------------------------------

        if objectCentreX < frameW*0.40

            leftOccupancy = ...
                leftOccupancy + areaRatio;

        elseif objectCentreX > frameW*0.60

            rightOccupancy = ...
                rightOccupancy + areaRatio;

        end

        %% -------------------------------------------------
        % CHECK WHETHER OBJECT ENTERS OUR FORWARD CORRIDOR
        %% -------------------------------------------------

        objectLeft  = x;
        objectRight = x+w;

        dangerLeft  = dangerRect(1);
        dangerRight = dangerRect(1)+dangerRect(3);

        overlapsForwardPath = ...
            objectRight > dangerLeft && ...
            objectLeft < dangerRight;

        %% -------------------------------------------------
        % APPROXIMATE PROXIMITY
        %
        % Larger + lower in image ≈ potentially closer.
        % This is NOT real metric distance.
        %% -------------------------------------------------

        nearObject = ...
            (areaRatio > 0.07) || ...
            (bottomRatio > 0.87 && areaRatio > 0.025);

        mediumObject = ...
            areaRatio > 0.018 && ...
            bottomRatio > 0.55;

        %% -------------------------------------------------
        % RISK
        %% -------------------------------------------------

        if overlapsForwardPath

            if nearObject

                highThreat = true;

                if labelStrings(i) == "person" || ...
                   labelStrings(i) == "bicycle" || ...
                   labelStrings(i) == "motorcycle"

                    vulnerableRoadUser = true;

                end

            elseif mediumObject

                mediumThreat = true;

            end

        end

    end

    %% =====================================================
    % DECISION MAKING
    %% =====================================================

    if highThreat

        risk = "HIGH";

        %% Pedestrian/bike near our path:
        % Prefer braking in this prototype

        if vulnerableRoadUser

            decision = "BRAKE";

        else

            %% Compare free space

            leftClear = ...
                leftOccupancy < 0.04;

            rightClear = ...
                rightOccupancy < 0.04;

            if leftClear && ~rightClear

                decision = "MOVE LEFT";

            elseif rightClear && ~leftClear

                decision = "MOVE RIGHT";

            elseif leftClear && rightClear

                %% Select less occupied side

                if leftOccupancy <= rightOccupancy

                    decision = "MOVE LEFT";

                else

                    decision = "MOVE RIGHT";

                end

            else

                decision = "BRAKE";

            end

        end

    elseif mediumThreat

        risk = "MEDIUM";
        decision = "SLOW";

    else

        risk = "SAFE";
        decision = "GO STRAIGHT";

    end

    %% =====================================================
    % CREATE 3D PLANNED PATH
    %% =====================================================

    pathX = linspace(8,45,100);
    pathY = zeros(size(pathX));

    switch decision

        case "MOVE LEFT"

            targetY = 3.5;

            for j = 1:length(pathX)

                t = (j-1)/(length(pathX)-1);

                pathY(j) = ...
                    targetY*smoothStep(t);

            end

        case "MOVE RIGHT"

            targetY = -3.5;

            for j = 1:length(pathX)

                t = (j-1)/(length(pathX)-1);

                pathY(j) = ...
                    targetY*smoothStep(t);

            end

        case "BRAKE"

            %% Short path indicates stopping

            pathX = linspace(8,18,40);
            pathY = zeros(size(pathX));

        otherwise

            pathY(:) = 0;

    end

    set(plannedPath, ...
        'XData',pathX, ...
        'YData',pathY, ...
        'ZData',0.15*ones(size(pathX)));

    %% =====================================================
    % DRAW YOLO BOXES
    %% =====================================================

    textLabels = strings(length(labelStrings),1);

    for i = 1:length(labelStrings)

        textLabels(i) = ...
            upper(labelStrings(i)) + ...
            " " + ...
            string(round(scores(i)*100)) + "%";

    end

    if ~isempty(bboxes)

        output = insertObjectAnnotation( ...
            frame, ...
            "rectangle", ...
            bboxes, ...
            cellstr(textLabels), ...
            'LineWidth',3);

    else

        output = frame;

    end

    %% Draw forward danger corridor

    output = insertShape( ...
        output, ...
        "Rectangle", ...
        dangerRect, ...
        'LineWidth',4);

    %% =====================================================
    % UPDATE CAMERA DISPLAY
    %% =====================================================

    set(cameraImage,'CData',output);

    title(axCamera, ...
        sprintf( ...
        ['LIVE YOLO PERCEPTION\n' ...
         'Objects: %d | Risk: %s'], ...
        length(labelStrings), ...
        risk), ...
        'FontWeight','bold');

    %% =====================================================
    % UPDATE 3D PANEL
    %% =====================================================

    title(ax3D, ...
        sprintf( ...
        ['PATH PLANNING\n' ...
         'Decision: %s'], ...
        decision), ...
        'FontWeight','bold');

    %% Overall dashboard title

    title(layout, ...
        sprintf( ...
        ['ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE\n' ...
         'Risk: %s     Decision: %s'], ...
        risk,decision), ...
        'FontWeight','bold');

    drawnow limitrate;

end

%% Release camera

clear cam;

disp("Camera released.");


%% =========================================================
% FUNCTIONS
%% =========================================================

function value = smoothStep(t)

value = t.^2 .* (3-2*t);

end


function h = createCuboid(ax,center,L,W,H,color)

x = center(1);
y = center(2);
z = center(3);

x1 = x-L/2;
x2 = x+L/2;

y1 = y-W/2;
y2 = y+W/2;

z1 = z-H/2;
z2 = z+H/2;

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