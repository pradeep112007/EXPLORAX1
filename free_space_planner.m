clear;
clc;
close all;

%% =========================================================
% PHASE 9
% LIVE CAMERA + YOLO + FREE SPACE PLANNER
%
% This is still a prototype.
% It does NOT estimate exact real-world distance.
%% =========================================================

disp("Loading YOLO...");
detector = yolov4ObjectDetector("tiny-yolov4-coco");

%% Connect camera
cam = webcam(1);

%% Read one frame first
frame = snapshot(cam);
[frameH,frameW,~] = size(frame);

%% =========================================================
% CAMERA ZONES
%% =========================================================

% Forward analysis area
roadTop    = round(frameH*0.28);
roadBottom = round(frameH*0.95);

zoneHeight = roadBottom - roadTop;

leftZone   = [1                  roadTop round(frameW/3)           zoneHeight];
centerZone = [round(frameW/3)+1  roadTop round(frameW/3)           zoneHeight];
rightZone  = [round(2*frameW/3)+1 roadTop frameW-round(2*frameW/3) zoneHeight];

%% =========================================================
% CREATE FIGURE
%% =========================================================

fig = figure( ...
    'Name','Free Space Planner Dashboard', ...
    'NumberTitle','off', ...
    'Color',[0.94 0.94 0.94]);

layout = tiledlayout(fig,1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% =========================================================
% LEFT PANEL - CAMERA
%% =========================================================

axCam = nexttile(layout,1);
camImg = imshow(frame,'Parent',axCam);
title(axCam,'LIVE CAMERA + FREE SPACE ANALYSIS');

%% =========================================================
% RIGHT PANEL - 3D PATH
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
    [0.28 0.55 0.24], ...
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
    'w','LineWidth',2);
plot3(ax3D,[0 50],[6 6],[0.1 0.1], ...
    'w','LineWidth',2);

%% Ego vehicle
createCuboid(ax3D,[6 0 0.65],4.5,2,1.3,[0.1 0.35 0.95]);
createCuboid(ax3D,[6 0 1.55],2.5,1.7,0.7,[0.05 0.15 0.40]);

%% Planned path
plannedPath = plot3(ax3D,nan,nan,nan, ...
    'LineWidth',4);

%% =========================================================
% MAIN LOOP
%% =========================================================

while ishandle(fig)

    frame = snapshot(cam);

    %% -----------------------------------------------------
    % YOLO DETECTION
    %% -----------------------------------------------------

    [bboxes,scores,labels] = detect( ...
        detector, ...
        frame, ...
        Threshold=0.35);

    labelStrings = string(labels);

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

    %% =====================================================
    % FREE SPACE VARIABLES
    %% =====================================================

    leftOcc = 0;
    centerOcc = 0;
    rightOcc = 0;

    highThreat = false;
    mediumThreat = false;
    vulnerableThreat = false;

    %% =====================================================
    % ANALYSE DETECTIONS
    %% =====================================================

    for i = 1:size(bboxes,1)

        box = bboxes(i,:);

        x = box(1);
        y = box(2);
        w = box(3);
        h = box(4);

        objCentreX = x + w/2;
        objBottom  = y + h;

        areaRatio = (w*h)/(frameW*frameH);
        bottomRatio = objBottom/frameH;

        %% Zone occupancy based on center point
        if objCentreX <= frameW/3
            leftOcc = leftOcc + areaRatio;
        elseif objCentreX <= 2*frameW/3
            centerOcc = centerOcc + areaRatio;
        else
            rightOcc = rightOcc + areaRatio;
        end

        %% Approximate closeness
        nearObject = ...
            (areaRatio > 0.07) || ...
            (bottomRatio > 0.85 && areaRatio > 0.025);

        mediumObject = ...
            areaRatio > 0.018 && ...
            bottomRatio > 0.55;

        %% If object is in centre area, it matters more
        if objCentreX > frameW/3 && objCentreX <= 2*frameW/3

            if nearObject
                highThreat = true;

                if labelStrings(i) == "person" || ...
                   labelStrings(i) == "bicycle" || ...
                   labelStrings(i) == "motorcycle"

                    vulnerableThreat = true;
                end

            elseif mediumObject
                mediumThreat = true;
            end
        end

    end

    %% =====================================================
    % DECISION LOGIC
    %% =====================================================

    risk = "SAFE";
    decision = "GO STRAIGHT";

    % free score = smaller occupancy = better
    leftFree   = 1 - leftOcc;
    centerFree = 1 - centerOcc;
    rightFree  = 1 - rightOcc;

    if highThreat

        risk = "HIGH";

        if vulnerableThreat
            decision = "BRAKE";
        else
            [~,idx] = max([leftFree centerFree rightFree]);

            if idx == 1
                decision = "MOVE LEFT";
            elseif idx == 3
                decision = "MOVE RIGHT";
            else
                decision = "BRAKE";
            end
        end

    elseif mediumThreat

        risk = "MEDIUM";

        if centerFree > 0.93
            decision = "SLOW";
        else
            if leftFree >= rightFree
                decision = "MOVE LEFT";
            else
                decision = "MOVE RIGHT";
            end
        end

    else

        risk = "SAFE";
        decision = "GO STRAIGHT";

    end

    %% =====================================================
    % MAKE 3D PATH
    %% =====================================================

    pathX = linspace(8,45,100);
    pathY = zeros(size(pathX));

    switch decision

        case "MOVE LEFT"
            targetY = 3.5;
            for j = 1:length(pathX)
                t = (j-1)/(length(pathX)-1);
                pathY(j) = targetY * smoothStep(t);
            end

        case "MOVE RIGHT"
            targetY = -3.5;
            for j = 1:length(pathX)
                t = (j-1)/(length(pathX)-1);
                pathY(j) = targetY * smoothStep(t);
            end

        case "BRAKE"
            pathX = linspace(8,18,35);
            pathY = zeros(size(pathX));

        otherwise
            pathY(:) = 0;
    end

    set(plannedPath, ...
        'XData',pathX, ...
        'YData',pathY, ...
        'ZData',0.15*ones(size(pathX)));

    %% =====================================================
    % DISPLAY LABELS
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

    %% =====================================================
    % DRAW LEFT / CENTER / RIGHT ZONES
    %% =====================================================

    output = insertShape(output,"Rectangle",leftZone, ...
        'LineWidth',3);
    output = insertShape(output,"Rectangle",centerZone, ...
        'LineWidth',3);
    output = insertShape(output,"Rectangle",rightZone, ...
        'LineWidth',3);

    output = insertText(output, ...
        [leftZone(1)+5 leftZone(2)+5], ...
        sprintf("LEFT\nOcc: %.2f",leftOcc), ...
        'FontSize',16, ...
        'BoxOpacity',0.6);

    output = insertText(output, ...
        [centerZone(1)+5 centerZone(2)+5], ...
        sprintf("CENTER\nOcc: %.2f",centerOcc), ...
        'FontSize',16, ...
        'BoxOpacity',0.6);

    output = insertText(output, ...
        [rightZone(1)+5 rightZone(2)+5], ...
        sprintf("RIGHT\nOcc: %.2f",rightOcc), ...
        'FontSize',16, ...
        'BoxOpacity',0.6);

    %% =====================================================
    % UPDATE CAMERA PANEL
    %% =====================================================

    set(camImg,'CData',output);

    title(axCam, sprintf( ...
        ['LIVE CAMERA\n' ...
         'Objects: %d | Risk: %s'], ...
        length(labelStrings), risk), ...
        'FontWeight','bold');

    %% =====================================================
    % UPDATE 3D PANEL
    %% =====================================================

    title(ax3D, sprintf( ...
        ['FREE SPACE PATH PLANNER\n' ...
         'Decision: %s'], ...
        decision), ...
        'FontWeight','bold');

    %% =====================================================
    % MAIN TITLE
    %% =====================================================

    title(layout, sprintf( ...
        ['ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE\n' ...
         'Risk: %s | Decision: %s | Left Free: %.2f | Center Free: %.2f | Right Free: %.2f'], ...
        risk, decision, leftFree, centerFree, rightFree), ...
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
value = t.^2 .* (3 - 2*t);
end

function h = createCuboid(ax,center,L,W,H,color)

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