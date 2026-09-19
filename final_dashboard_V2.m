function final_dashboard_v2

clc;

%% =========================================================
% ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE
% Live Camera / Video + YOLO + Decision Logic + Path Planner
%
% SOFTWARE / SIMULATION PROTOTYPE ONLY
%% =========================================================

%% SHARED VARIABLES

cam = [];
videoReader = [];
detector = [];
timerObj = [];

inputMode = "NONE";
running = false;

egoX = 8;
egoY = 0;

avoidanceMode = "NONE";
avoidanceCounter = 0;
avoidanceDuration = 25;

%% =========================================================
% MAIN WINDOW
%% =========================================================

app = uifigure( ...
    'Name','Adaptive Path Planning & Collision Avoidance', ...
    'Position',[40 30 1500 880], ...
    'Color',[0.96 0.98 1], ...
    'CloseRequestFcn',@closeApp);

mainGrid = uigridlayout(app,[5 2]);

mainGrid.RowHeight = {70,60,'1x',110,50};
mainGrid.ColumnWidth = {'1.25x','1x'};

mainGrid.Padding = [14 14 14 14];

mainGrid.RowSpacing = 10;
mainGrid.ColumnSpacing = 10;

%% =========================================================
% HEADER
%% =========================================================

header = uipanel(mainGrid);

header.Layout.Row = 1;
header.Layout.Column = [1 2];

header.BackgroundColor = [1 1 1];

headerGrid = uigridlayout(header,[1 3]);

headerGrid.ColumnWidth = {250,'1x',250};

brand = uilabel(headerGrid);

brand.Text = sprintf( ...
    'AUTONOMOUS DRIVING LAB\nMATLAB Prototype');

brand.FontWeight = 'bold';
brand.FontSize = 13;

brand.FontColor = [0.05 0.35 0.70];

titleLabel = uilabel(headerGrid);

titleLabel.Text = sprintf( ...
    ['ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE\n' ...
    'For Autonomous Vehicles on Unstructured Indian Roads']);

titleLabel.HorizontalAlignment = 'center';

titleLabel.FontSize = 21;
titleLabel.FontWeight = 'bold';

titleLabel.FontColor = [0.03 0.42 0.85];

project = uilabel(headerGrid);

project.Text = sprintf( ...
    'CSE PROJECT\nAI Driving System');

project.HorizontalAlignment = 'right';

project.FontWeight = 'bold';

project.FontColor = [0.25 0.35 0.50];

%% =========================================================
% CONTROL BAR
%% =========================================================

controlPanel = uipanel(mainGrid);

controlPanel.Layout.Row = 2;
controlPanel.Layout.Column = [1 2];

controlPanel.BackgroundColor = [0.985 0.99 1];

controls = uigridlayout(controlPanel,[1 7]);

controls.ColumnWidth = ...
    {'1x','1x','1.2x','1x','1x','1x','0.9x'};

liveButton = uibutton(controls,'push');

liveButton.Text = 'LIVE CAMERA';
liveButton.FontWeight = 'bold';

videoButton = uibutton(controls,'push');

videoButton.Text = 'LOAD ROAD VIDEO';
videoButton.FontWeight = 'bold';

scenarioDrop = uidropdown(controls);

scenarioDrop.Items = { ...
    'Live Camera', ...
    'Unmarked Village Road', ...
    'Urban Unsignalised Intersection', ...
    'Highway Slow-Vehicle Merge', ...
    'Dense Market Area', ...
    'Sudden Cattle Crossing'};

scenarioDrop.Value = 'Live Camera';

scenarioButton = uibutton(controls,'push');

scenarioButton.Text = 'SCENARIO';
scenarioButton.FontWeight = 'bold';

startButton = uibutton(controls,'push');

startButton.Text = 'START AI';
startButton.FontWeight = 'bold';

stopButton = uibutton(controls,'push');

stopButton.Text = 'STOP';
stopButton.FontWeight = 'bold';

resetButton = uibutton(controls,'push');

resetButton.Text = 'RESET';
resetButton.FontWeight = 'bold';

%% =========================================================
% CAMERA PANEL
%% =========================================================

cameraPanel = uipanel(mainGrid);

cameraPanel.Layout.Row = 3;
cameraPanel.Layout.Column = 1;

cameraPanel.Title = 'LIVE AI PERCEPTION';

cameraPanel.FontWeight = 'bold';

cameraPanel.ForegroundColor = [0.03 0.40 0.82];

cameraPanel.BackgroundColor = [1 1 1];

cameraGrid = uigridlayout(cameraPanel,[2 1]);

cameraGrid.RowHeight = {'1x',90};

cameraAxes = uiaxes(cameraGrid);

cameraAxes.Layout.Row = 1;

cameraAxes.XTick = [];
cameraAxes.YTick = [];

cameraAxes.Toolbar.Visible = 'off';

cameraAxes.Color = [0.94 0.96 0.98];

blank = zeros(480,720,3,'uint8');

blank(:,:,1) = 238;
blank(:,:,2) = 245;
blank(:,:,3) = 252;

cameraImage = imshow(blank,'Parent',cameraAxes);

title(cameraAxes, ...
    'Select LIVE CAMERA or LOAD ROAD VIDEO', ...
    'FontWeight','bold');

%% ANALYSIS CARDS

analysisGrid = uigridlayout(cameraGrid,[1 3]);

analysisGrid.Layout.Row = 2;

analysisGrid.ColumnWidth = {'1x','1x','1x'};

leftCard = createAnalysisCard( ...
    analysisGrid, ...
    1, ...
    'LEFT ANALYSIS');

centerCard = createAnalysisCard( ...
    analysisGrid, ...
    2, ...
    'CENTER ANALYSIS');

rightCard = createAnalysisCard( ...
    analysisGrid, ...
    3, ...
    'RIGHT ANALYSIS');

%% =========================================================
% PATH PLANNER
%% =========================================================

plannerPanel = uipanel(mainGrid);

plannerPanel.Layout.Row = 3;
plannerPanel.Layout.Column = 2;

plannerPanel.Title = 'ADAPTIVE PATH PLANNER';

plannerPanel.FontWeight = 'bold';

plannerPanel.ForegroundColor = [0.03 0.40 0.82];

plannerPanel.BackgroundColor = [1 1 1];

plannerGrid = uigridlayout(plannerPanel,[1 1]);

plannerAxes = uiaxes(plannerGrid);

hold(plannerAxes,'on');

plannerAxes.Toolbar.Visible = 'off';

plannerAxes.Color = [0.94 0.96 0.98];

xlim(plannerAxes,[0 100]);

ylim(plannerAxes,[-14 14]);

xlabel(plannerAxes,'Forward Distance');

ylabel(plannerAxes,'Road Width');

title(plannerAxes, ...
    'Top-Down View / Adaptive Path', ...
    'FontWeight','bold');

grid(plannerAxes,'on');

%% ROAD

rectangle( ...
    plannerAxes, ...
    'Position',[0 -6 100 12], ...
    'FaceColor',[0.30 0.33 0.38], ...
    'EdgeColor',[0.55 0.60 0.68]);

plot(plannerAxes,[0 100],[-6 -6], ...
    'LineWidth',2);

plot(plannerAxes,[0 100],[6 6], ...
    'LineWidth',2);

%% EGO CAR

egoVehicle = rectangle( ...
    plannerAxes, ...
    'Position',[egoX-2.8 egoY-1.1 5.6 2.2], ...
    'Curvature',0.25, ...
    'FaceColor',[0.05 0.55 0.95], ...
    'EdgeColor',[0.75 0.90 1], ...
    'LineWidth',2);

egoText = text( ...
    plannerAxes, ...
    egoX, ...
    egoY-2.2, ...
    'EGO', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold');

%% OBSTACLE

plannerObstacle = rectangle( ...
    plannerAxes, ...
    'Position',[55 -1.2 5.5 2.4], ...
    'Curvature',0.2, ...
    'FaceColor',[0.95 0.20 0.20], ...
    'Visible','off');

obstacleText = text( ...
    plannerAxes, ...
    58, ...
    2.2, ...
    'OBJECT', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold', ...
    'Visible','off');

%% SAFE PATH

pathPlot = plot( ...
    plannerAxes, ...
    linspace(egoX+3,95,100), ...
    zeros(1,100), ...
    'LineWidth',4);

%% =========================================================
% STATUS PANEL
%% =========================================================

statusPanel = uipanel(mainGrid);

statusPanel.Layout.Row = 4;

statusPanel.Layout.Column = [1 2];

statusPanel.Title = 'REAL-TIME ANALYTICS & STATUS';

statusPanel.FontWeight = 'bold';

statusPanel.ForegroundColor = [0.03 0.40 0.82];

statusPanel.BackgroundColor = [1 1 1];

statusGrid = uigridlayout(statusPanel,[2 8]);

statusGrid.RowHeight = {30,50};

statusGrid.ColumnWidth = ...
    {'1x','1x','1x','1x','1x','1x','1x','1x'};

headers = { ...
    'OBJECT', ...
    'RISK LEVEL', ...
    'DECISION', ...
    'SPEED', ...
    'REPLAN LATENCY', ...
    'COLLISION', ...
    'PATH SMOOTHNESS', ...
    'COMPLETION'};

for i = 1:8

    h = uilabel(statusGrid);

    h.Layout.Row = 1;

    h.Layout.Column = i;

    h.Text = headers{i};

    h.HorizontalAlignment = 'center';

    h.FontWeight = 'bold';

    h.FontColor = [0.35 0.45 0.60];

end

objectValue = makeValue( ...
    statusGrid,1,'NONE');

riskValue = makeValue( ...
    statusGrid,2,'SAFE');

decisionValue = makeValue( ...
    statusGrid,3,'GO STRAIGHT');

speedValue = makeValue( ...
    statusGrid,4,'30 km/h');

latencyValue = makeValue( ...
    statusGrid,5,'-- ms');

collisionValue = makeValue( ...
    statusGrid,6,'NO');

smoothnessValue = makeValue( ...
    statusGrid,7,'--');

completionValue = makeValue( ...
    statusGrid,8,'--');

%% =========================================================
% FOOTER
%% =========================================================

footer = uipanel(mainGrid);

footer.Layout.Row = 5;

footer.Layout.Column = [1 2];

footer.BackgroundColor = [0.985 0.99 1];

footerGrid = uigridlayout(footer,[1 5]);

footerGrid.ColumnWidth = ...
    {'1.3x','1.1x','1x','1x','1.2x'};

scenarioLabel = footerText( ...
    footerGrid,1,'Scenario: Live Camera');

inputLabel = footerText( ...
    footerGrid,2,'Input: Not Selected');

trafficLabel = footerText( ...
    footerGrid,3,'Traffic: --');

statusLabel = footerText( ...
    footerGrid,4,'System: READY');

systemLabel = footerText( ...
    footerGrid,5,'SYSTEM STATUS: READY');

systemLabel.FontColor = [0 0.60 0.40];

%% =========================================================
% BUTTON CALLBACKS
%% =========================================================

liveButton.ButtonPushedFcn = @selectCamera;

videoButton.ButtonPushedFcn = @selectVideo;

scenarioButton.ButtonPushedFcn = @scenarioSelected;

startButton.ButtonPushedFcn = @startSystem;

stopButton.ButtonPushedFcn = @stopSystem;

resetButton.ButtonPushedFcn = @resetSystem;

%% =========================================================
% LIVE CAMERA
%% =========================================================

    function selectCamera(~,~)

        stopSystem();

        try

            if isempty(cam)

                cam = webcam(1);

            end

            frame = snapshot(cam);

            cameraImage.CData = frame;

            inputMode = "CAMERA";

            inputLabel.Text = ...
                'Input: Live Camera';

            scenarioLabel.Text = ...
                'Scenario: Live Camera';

            statusLabel.Text = ...
                'System: CAMERA READY';

            systemLabel.Text = ...
                'SYSTEM STATUS: READY';

        catch ME

            uialert( ...
                app, ...
                ME.message, ...
                'Camera Error');

        end

    end

%% =========================================================
% VIDEO INPUT
%% =========================================================

    function selectVideo(~,~)

        stopSystem();

        [file,path] = uigetfile( ...
            {'*.mp4;*.mov;*.avi','Road Videos'}, ...
            'Select Road Video');

        if isequal(file,0)

            return;

        end

        try

            videoReader = ...
                VideoReader(fullfile(path,file));

            frame = readFrame(videoReader);

            cameraImage.CData = frame;

            videoReader.CurrentTime = 0;

            inputMode = "VIDEO";

            inputLabel.Text = ...
                'Input: Road Video';

            scenarioLabel.Text = ...
                'Scenario: Video Analysis';

            statusLabel.Text = ...
                'System: VIDEO READY';

        catch ME

            uialert( ...
                app, ...
                ME.message, ...
                'Video Error');

        end

    end

%% =========================================================
% SCENARIO
%% =========================================================

    function scenarioSelected(~,~)

        selected = string(scenarioDrop.Value);

        scenarioLabel.Text = ...
            "Scenario: " + selected;

        switch selected

            case "Dense Market Area"

                trafficLabel.Text = ...
                    'Traffic: HIGH';

            case "Urban Unsignalised Intersection"

                trafficLabel.Text = ...
                    'Traffic: IRREGULAR';

            case "Highway Slow-Vehicle Merge"

                trafficLabel.Text = ...
                    'Traffic: MERGING';

            case "Sudden Cattle Crossing"

                trafficLabel.Text = ...
                    'Traffic: ANIMAL RISK';

            case "Unmarked Village Road"

                trafficLabel.Text = ...
                    'Traffic: MIXED';

            otherwise

                trafficLabel.Text = ...
                    'Traffic: LIVE';

        end

    end

%% =========================================================
% START AI
%% =========================================================

    function startSystem(~,~)

        if running

            return;

        end

        if inputMode == "NONE"

            uialert( ...
                app, ...
                'Choose LIVE CAMERA or LOAD ROAD VIDEO first.', ...
                'Input Required');

            return;

        end

        if isempty(detector)

            statusLabel.Text = ...
                'System: Loading YOLO...';

            drawnow;

            try

                detector = ...
                    yolov4ObjectDetector( ...
                    "tiny-yolov4-coco");

            catch ME

                uialert( ...
                    app, ...
                    ME.message, ...
                    'YOLO Error');

                return;

            end

        end

        running = true;

        statusLabel.Text = ...
            'System: RUNNING';

        systemLabel.Text = ...
            'SYSTEM STATUS: RUNNING';

        if isempty(timerObj) || ...
                ~isvalid(timerObj)

            timerObj = timer( ...
                'ExecutionMode','fixedSpacing', ...
                'Period',0.20, ...
                'BusyMode','drop', ...
                'TimerFcn',@processFrame);

        end

        start(timerObj);

    end

%% =========================================================
% STOP
%% =========================================================

    function stopSystem(varargin)

        running = false;

        if ~isempty(timerObj)

            try

                if isvalid(timerObj) && ...
                        strcmp(timerObj.Running,'on')

                    stop(timerObj);

                end

            catch
            end

        end

        statusLabel.Text = ...
            'System: STOPPED';

        systemLabel.Text = ...
            'SYSTEM STATUS: STOPPED';

    end

%% =========================================================
% RESET
%% =========================================================

    function resetSystem(~,~)

        stopSystem();

        egoX = 8;
        egoY = 0;

        avoidanceMode = "NONE";
        avoidanceCounter = 0;

        egoVehicle.Position = ...
            [egoX-2.8 egoY-1.1 5.6 2.2];

        egoText.Position = ...
            [egoX egoY-2.2 0];

        plannerObstacle.Visible = 'off';

        obstacleText.Visible = 'off';

        pathPlot.XData = ...
            linspace(egoX+3,95,100);

        pathPlot.YData = ...
            zeros(1,100);

        objectValue.Text = 'NONE';

        riskValue.Text = 'SAFE';

        decisionValue.Text = ...
            'GO STRAIGHT';

        speedValue.Text = ...
            '30 km/h';

        latencyValue.Text = ...
            '-- ms';

        collisionValue.Text = ...
            'NO';

        smoothnessValue.Text = ...
            '--';

        completionValue.Text = ...
            '--';

        leftCard.Text = ...
            'Waiting for AI...';

        centerCard.Text = ...
            'Waiting for AI...';

        rightCard.Text = ...
            'Waiting for AI...';

        systemLabel.Text = ...
            'SYSTEM STATUS: READY';

        statusLabel.Text = ...
            'System: READY';

    end

%% =========================================================
% PROCESS FRAME
%% =========================================================

    function processFrame(~,~)

        if ~running || ~isvalid(app)

            return;

        end

        startTime = tic;

        %% GET FRAME

        try

            if inputMode == "CAMERA"

                frame = snapshot(cam);

            elseif inputMode == "VIDEO"

                if hasFrame(videoReader)

                    frame = ...
                        readFrame(videoReader);

                else

                    videoReader.CurrentTime = 0;

                    frame = ...
                        readFrame(videoReader);

                end

            else

                return;

            end

        catch

            return;

        end

        [frameH,frameW,~] = ...
            size(frame);

        %% YOLO

        try

            [bboxes,scores,labels] = ...
                detect( ...
                detector, ...
                frame, ...
                Threshold=0.35);

        catch

            return;

        end

        labels = string(labels);

        classes = [ ...
            "person", ...
            "car", ...
            "motorcycle", ...
            "bicycle", ...
            "bus", ...
            "truck"];

        keep = ...
            ismember(labels,classes);

        bboxes = ...
            bboxes(keep,:);

        scores = ...
            scores(keep);

        labels = ...
            labels(keep);

        %% =================================================
        % ZONE ANALYSIS
        %% =================================================

        third = frameW/3;

        leftOcc = 0;
        centerOcc = 0;
        rightOcc = 0;

        vehicleThreat = false;

        vulnerableThreat = false;

        criticalThreat = false;

        primaryObject = "NONE";

        bestScore = 0;

        for i = 1:size(bboxes,1)

            box = bboxes(i,:);

            x = box(1);
            y = box(2);
            w = box(3);
            h = box(4);

            centerX = x + w/2;

            bottomY = y + h;

            areaRatio = ...
                (w*h)/(frameW*frameH);

            bottomRatio = ...
                bottomY/frameH;

            if scores(i) > bestScore

                bestScore = scores(i);

                primaryObject = ...
                    upper(labels(i));

            end

            if centerX < third

                leftOcc = ...
                    leftOcc + areaRatio;

            elseif centerX < 2*third

                centerOcc = ...
                    centerOcc + areaRatio;

            else

                rightOcc = ...
                    rightOcc + areaRatio;

            end

            objectInCenter = ...
                centerX > third && ...
                centerX < 2*third;

            vulnerable = ...
                labels(i) == "person" || ...
                labels(i) == "bicycle" || ...
                labels(i) == "motorcycle";

            vehicle = ...
                labels(i) == "car" || ...
                labels(i) == "bus" || ...
                labels(i) == "truck";

            criticalObject = ...
                areaRatio > 0.14 && ...
                bottomRatio > 0.78;

            closeObject = ...
                areaRatio > 0.050 && ...
                bottomRatio > 0.58;

            mediumObject = ...
                areaRatio > 0.012 && ...
                bottomRatio > 0.42;

            if objectInCenter

                if vehicle && ...
                        (closeObject || ...
                        mediumObject)

                    vehicleThreat = true;

                end

                if vulnerable && ...
                        (closeObject || ...
                        mediumObject)

                    vulnerableThreat = true;

                end

                if vulnerable && ...
                        criticalObject

                    criticalThreat = true;

                end

            end

        end

        %% FREE SPACE

        leftFree = ...
            max(0,1-leftOcc);

        centerFree = ...
            max(0,1-centerOcc);

        rightFree = ...
            max(0,1-rightOcc);

        %% =================================================
        % DECISION LOGIC
        %% =================================================

        risk = "SAFE";

        decision = ...
            "GO STRAIGHT";

        if criticalThreat

            risk = "CRITICAL";

            decision = "BRAKE";

            avoidanceMode = "NONE";

            avoidanceCounter = 0;

        elseif avoidanceMode ~= "NONE"

            risk = "HIGH";

            decision = avoidanceMode;

            avoidanceCounter = ...
                avoidanceCounter - 1;

            if avoidanceCounter <= 0

                avoidanceMode = "NONE";

            end

        elseif vehicleThreat

            risk = "HIGH";

            if leftFree > ...
                    rightFree + 0.02

                decision = ...
                    "MOVE LEFT";

            elseif rightFree > ...
                    leftFree + 0.02

                decision = ...
                    "MOVE RIGHT";

            else

                decision = ...
                    "MOVE LEFT";

            end

            avoidanceMode = ...
                decision;

            avoidanceCounter = ...
                avoidanceDuration;

        elseif vulnerableThreat

            risk = "MEDIUM";

            decision = "SLOW";

        end

        %% =================================================
        % SIMULATED VEHICLE MOVEMENT
        %% =================================================

        switch decision

            case "GO STRAIGHT"

                targetY = 0;

                forwardStep = 0.55;

                speed = 30;

            case "SLOW"

                targetY = 0;

                forwardStep = 0.22;

                speed = 12;

            case "MOVE LEFT"

                targetY = 4;

                forwardStep = 0.42;

                speed = 22;

            case "MOVE RIGHT"

                targetY = -4;

                forwardStep = 0.42;

                speed = 22;

            case "BRAKE"

                targetY = egoY;

                forwardStep = 0;

                speed = 0;

        end

        lateralError = ...
            targetY - egoY;

        sideStep = max( ...
            min(lateralError,0.22), ...
            -0.22);

        egoY = ...
            egoY + sideStep;

        egoX = ...
            egoX + forwardStep;

        if egoX > 91

            egoX = 8;

            egoY = 0;

            avoidanceMode = "NONE";

        end

        egoVehicle.Position = ...
            [egoX-2.8 ...
            egoY-1.1 ...
            5.6 ...
            2.2];

        egoText.Position = ...
            [egoX ...
            egoY-2.2 ...
            0];

        %% =================================================
        % DISPLAY OBSTACLE
        %% =================================================

        if vehicleThreat || ...
                vulnerableThreat || ...
                criticalThreat

            plannerObstacle.Visible = ...
                'on';

            obstacleText.Visible = ...
                'on';

            obstacleX = ...
                min(85,egoX+25);

            plannerObstacle.Position = ...
                [obstacleX-2.7 ...
                -1.2 ...
                5.4 ...
                2.4];

            obstacleText.Position = ...
                [obstacleX ...
                2.3 ...
                0];

            obstacleText.String = ...
                primaryObject;

        else

            plannerObstacle.Visible = ...
                'off';

            obstacleText.Visible = ...
                'off';

        end

        %% =================================================
        % PATH
        %% =================================================

        xEnd = ...
            min(98,egoX+48);

        px = linspace( ...
            egoX+3, ...
            xEnd, ...
            120);

        switch decision

            case "MOVE LEFT"

                py = makeAvoidancePath( ...
                    px, ...
                    egoY, ...
                    4);

            case "MOVE RIGHT"

                py = makeAvoidancePath( ...
                    px, ...
                    egoY, ...
                    -4);

            case "BRAKE"

                px = linspace( ...
                    egoX+3, ...
                    min(98,egoX+10), ...
                    30);

                py = ...
                    egoY * ...
                    ones(size(px));

            otherwise

                py = makeReturnPath( ...
                    px, ...
                    egoY);

        end

        pathPlot.XData = px;

        pathPlot.YData = py;

        %% PATH SMOOTHNESS

        if length(py) > 2

            smoothness = ...
                max( ...
                0, ...
                min( ...
                1, ...
                1 - ...
                mean( ...
                abs(diff(py,2)))));

        else

            smoothness = 1;

        end

        %% =================================================
        % YOLO BOXES
        %% =================================================

        if ~isempty(bboxes)

            texts = ...
                strings(length(labels),1);

            for i = 1:length(labels)

                texts(i) = ...
                    upper(labels(i)) + ...
                    " " + ...
                    string( ...
                    round( ...
                    scores(i)*100)) + ...
                    "%";

            end

            output = ...
                insertObjectAnnotation( ...
                frame, ...
                "rectangle", ...
                bboxes, ...
                cellstr(texts), ...
                'LineWidth',3);

        else

            output = frame;

        end

        %% CAMERA ZONES

        top = round(frameH*0.30);

        zoneHeight = ...
            round(frameH*0.65);

        leftRect = ...
            [1 ...
            top ...
            round(third) ...
            zoneHeight];

        centerRect = ...
            [round(third)+1 ...
            top ...
            round(third) ...
            zoneHeight];

        rightRect = ...
            [round(2*third)+1 ...
            top ...
            frameW-round(2*third) ...
            zoneHeight];

        output = insertShape( ...
            output, ...
            "Rectangle", ...
            leftRect, ...
            'LineWidth',2);

        output = insertShape( ...
            output, ...
            "Rectangle", ...
            centerRect, ...
            'LineWidth',3);

        output = insertShape( ...
            output, ...
            "Rectangle", ...
            rightRect, ...
            'LineWidth',2);

        %% =================================================
        % UI UPDATE
        %% =================================================

        cameraImage.CData = output;

        objectValue.Text = ...
            primaryObject;

        riskValue.Text = ...
            risk;

        decisionValue.Text = ...
            decision;

        speedValue.Text = ...
            string(speed) + ...
            " km/h";

        latency = ...
            toc(startTime)*1000;

        latencyValue.Text = ...
            sprintf( ...
            '%.0f ms', ...
            latency);

        collisionValue.Text = ...
            'NO';

        smoothnessValue.Text = ...
            sprintf( ...
            '%.2f', ...
            smoothness);

        completion = ...
            min( ...
            100, ...
            egoX/91*100);

        completionValue.Text = ...
            sprintf( ...
            '%.0f%%', ...
            completion);

        leftCard.Text = ...
            sprintf( ...
            'Free space %.0f%%', ...
            leftFree*100);

        centerCard.Text = ...
            sprintf( ...
            'Free space %.0f%%', ...
            centerFree*100);

        rightCard.Text = ...
            sprintf( ...
            'Free space %.0f%%', ...
            rightFree*100);

        if length(labels) >= 4

            trafficLabel.Text = ...
                'Traffic: HIGH';

        elseif length(labels) >= 2

            trafficLabel.Text = ...
                'Traffic: MEDIUM';

        else

            trafficLabel.Text = ...
                'Traffic: LOW';

        end

        drawnow limitrate;

    end

%% =========================================================
% CLOSE APPLICATION
%% =========================================================

    function closeApp(~,~)

        stopSystem();

        if ~isempty(timerObj)

            try

                if isvalid(timerObj)

                    delete(timerObj);

                end

            catch
            end

        end

        delete(app);

    end

end


%% =========================================================
% HELPER FUNCTIONS
%% =========================================================

function valueLabel = createAnalysisCard(parent,column,titleText)

panel = uipanel(parent);

panel.Layout.Column = column;

panel.BackgroundColor = ...
    [0.97 0.985 1];

grid = uigridlayout(panel,[2 1]);

grid.RowHeight = {30,'1x'};

titleLabel = uilabel(grid);

titleLabel.Text = titleText;

titleLabel.FontWeight = 'bold';

titleLabel.FontColor = ...
    [0.03 0.40 0.82];

valueLabel = uilabel(grid);

valueLabel.Text = ...
    'Waiting for AI...';

valueLabel.FontColor = ...
    [0.25 0.35 0.50];

end


function h = makeValue(parent,column,textValue)

h = uilabel(parent);

h.Layout.Row = 2;

h.Layout.Column = column;

h.Text = textValue;

h.HorizontalAlignment = ...
    'center';

h.FontWeight = ...
    'bold';

h.FontSize = 14;

h.FontColor = ...
    [0.08 0.25 0.45];

end


function h = footerText(parent,column,textValue)

h = uilabel(parent);

h.Layout.Column = column;

h.Text = textValue;

h.FontWeight = 'bold';

h.FontColor = ...
    [0.20 0.35 0.50];

end


function y = smoothStep(t)

y = ...
    t.^2 .* ...
    (3 - 2*t);

end


function py = makeAvoidancePath(px,currentY,targetY)

n = length(px);

py = zeros(size(px));

for k = 1:n

    t = ...
        (k-1)/(n-1);

    if t < 0.35

        u = t/0.35;

        py(k) = ...
            currentY + ...
            (targetY-currentY) * ...
            smoothStep(u);

    elseif t < 0.70

        py(k) = targetY;

    else

        u = ...
            (t-0.70)/0.30;

        py(k) = ...
            targetY * ...
            (1-smoothStep(u));

    end

end

end


function py = makeReturnPath(px,currentY)

n = length(px);

py = zeros(size(px));

for k = 1:n

    t = ...
        (k-1)/(n-1);

    py(k) = ...
        currentY * ...
        (1-smoothStep(t));

end

end