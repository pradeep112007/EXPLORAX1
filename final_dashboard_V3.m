function final_dashboard_v3

clc;

%% =========================================================
% FINAL DASHBOARD V3
% Adaptive Path Planning & Collision Avoidance
% Unstructured Indian Roads
%
% FEATURES
% - Light UI
% - Live webcam
% - Road video input
% - YOLO v4 detection
% - Left / Center / Right analysis
% - Vehicle / pedestrian risk logic
% - Move Left / Move Right / Slow / Brake
% - Realistic top-down ego car
% - Realistic detected vehicle
% - Moving vehicle animation
% - Adaptive path visualization
% - Replanning latency
% - Path smoothness
%
% SOFTWARE / SIMULATION PROTOTYPE ONLY
%% =========================================================


%% =========================================================
% SHARED STATE
%% =========================================================

cam = [];

videoReader = [];

detector = [];

timerObj = [];

inputMode = "NONE";

running = false;


%% Ego vehicle state

egoX = 8;

egoY = 0;


%% Avoidance memory

avoidanceMode = "NONE";

avoidanceCounter = 0;

avoidanceDuration = 25;


%% =========================================================
% MAIN WINDOW
%% =========================================================

app = uifigure( ...
    'Name', ...
    'Adaptive Path Planning & Collision Avoidance', ...
    'Position', ...
    [30 30 1510 880], ...
    'Color', ...
    [0.96 0.98 1], ...
    'CloseRequestFcn', ...
    @closeApp);


%% Force dashboard into light mode

try

    theme(app,"light");

catch

end


%% =========================================================
% MAIN GRID
%% =========================================================

mainGrid = uigridlayout(app,[5 2]);

mainGrid.RowHeight = ...
    {72,58,'1x',112,50};

mainGrid.ColumnWidth = ...
    {'1.25x','1x'};

mainGrid.Padding = ...
    [14 14 14 14];

mainGrid.RowSpacing = 10;

mainGrid.ColumnSpacing = 10;


%% =========================================================
% HEADER
%% =========================================================

headerPanel = uipanel(mainGrid);

headerPanel.Layout.Row = 1;

headerPanel.Layout.Column = [1 2];

headerPanel.BackgroundColor = [1 1 1];


headerGrid = ...
    uigridlayout(headerPanel,[1 3]);

headerGrid.ColumnWidth = ...
    {260,'1x',260};


%% Brand

brandLabel = uilabel(headerGrid);

brandLabel.Text = sprintf( ...
    ['AUTONOMOUS DRIVING LAB\n' ...
     'MATLAB Prototype']);

brandLabel.FontSize = 13;

brandLabel.FontWeight = 'bold';

brandLabel.FontColor = ...
    [0.02 0.38 0.78];


%% Main title

mainTitle = uilabel(headerGrid);

mainTitle.Text = sprintf( ...
    ['ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE\n' ...
     'For Autonomous Vehicles on Unstructured Indian Roads']);

mainTitle.FontSize = 21;

mainTitle.FontWeight = 'bold';

mainTitle.FontColor = ...
    [0.02 0.42 0.88];

mainTitle.HorizontalAlignment = ...
    'center';


%% Project label

projectLabel = uilabel(headerGrid);

projectLabel.Text = sprintf( ...
    ['CSE PROJECT\n' ...
     'AI Driving Prototype']);

projectLabel.HorizontalAlignment = ...
    'right';

projectLabel.FontWeight = 'bold';

projectLabel.FontColor = ...
    [0.25 0.38 0.55];


%% =========================================================
% CONTROL PANEL
%% =========================================================

controlPanel = uipanel(mainGrid);

controlPanel.Layout.Row = 2;

controlPanel.Layout.Column = [1 2];

controlPanel.BackgroundColor = ...
    [0.985 0.99 1];


controls = ...
    uigridlayout(controlPanel,[1 7]);

controls.ColumnWidth = ...
    {'1x','1x','1.2x','1x','1x','1x','0.9x'};


%% LIVE CAMERA

liveButton = ...
    uibutton(controls,'push');

liveButton.Text = ...
    'LIVE CAMERA';

liveButton.FontWeight = ...
    'bold';


%% VIDEO

videoButton = ...
    uibutton(controls,'push');

videoButton.Text = ...
    'LOAD ROAD VIDEO';

videoButton.FontWeight = ...
    'bold';


%% Scenario selector

scenarioDrop = ...
    uidropdown(controls);

scenarioDrop.Items = { ...
    'Live Camera', ...
    'Unmarked Village Road', ...
    'Urban Unsignalised Intersection', ...
    'Highway Slow-Vehicle Merge', ...
    'Dense Market Area', ...
    'Sudden Cattle Crossing'};

scenarioDrop.Value = ...
    'Live Camera';


scenarioButton = ...
    uibutton(controls,'push');

scenarioButton.Text = ...
    'SCENARIO';

scenarioButton.FontWeight = ...
    'bold';


%% Start

startButton = ...
    uibutton(controls,'push');

startButton.Text = ...
    'START AI';

startButton.FontWeight = ...
    'bold';


%% Stop

stopButton = ...
    uibutton(controls,'push');

stopButton.Text = ...
    'STOP';

stopButton.FontWeight = ...
    'bold';


%% Reset

resetButton = ...
    uibutton(controls,'push');

resetButton.Text = ...
    'RESET';

resetButton.FontWeight = ...
    'bold';


%% =========================================================
% CAMERA PANEL
%% =========================================================

cameraPanel = ...
    uipanel(mainGrid);

cameraPanel.Layout.Row = 3;

cameraPanel.Layout.Column = 1;

cameraPanel.Title = ...
    'LIVE AI PERCEPTION';

cameraPanel.FontWeight = ...
    'bold';

cameraPanel.ForegroundColor = ...
    [0.02 0.40 0.85];

cameraPanel.BackgroundColor = ...
    [1 1 1];


cameraLayout = ...
    uigridlayout(cameraPanel,[2 1]);

cameraLayout.RowHeight = ...
    {'1x',90};


%% Camera axes

cameraAxes = ...
    uiaxes(cameraLayout);

cameraAxes.Layout.Row = 1;

cameraAxes.XTick = [];

cameraAxes.YTick = [];

cameraAxes.Toolbar.Visible = ...
    'off';

cameraAxes.Color = ...
    [0.94 0.97 1];


%% Blank camera image

blank = ...
    zeros(480,720,3,'uint8');

blank(:,:,1) = 238;

blank(:,:,2) = 245;

blank(:,:,3) = 252;


cameraImage = ...
    imshow( ...
    blank, ...
    'Parent',cameraAxes);


title( ...
    cameraAxes, ...
    'Select LIVE CAMERA or LOAD ROAD VIDEO', ...
    'FontWeight','bold');


%% =========================================================
% LEFT/CENTER/RIGHT ANALYSIS
%% =========================================================

analysisGrid = ...
    uigridlayout(cameraLayout,[1 3]);

analysisGrid.Layout.Row = 2;

analysisGrid.ColumnWidth = ...
    {'1x','1x','1x'};


leftCard = ...
    createAnalysisCard( ...
    analysisGrid, ...
    1, ...
    'LEFT ANALYSIS');


centerCard = ...
    createAnalysisCard( ...
    analysisGrid, ...
    2, ...
    'CENTER ANALYSIS');


rightCard = ...
    createAnalysisCard( ...
    analysisGrid, ...
    3, ...
    'RIGHT ANALYSIS');


%% =========================================================
% PATH PLANNER PANEL
%% =========================================================

plannerPanel = ...
    uipanel(mainGrid);

plannerPanel.Layout.Row = 3;

plannerPanel.Layout.Column = 2;

plannerPanel.Title = ...
    'ADAPTIVE PATH PLANNER';

plannerPanel.FontWeight = ...
    'bold';

plannerPanel.ForegroundColor = ...
    [0.02 0.40 0.85];

plannerPanel.BackgroundColor = ...
    [1 1 1];


plannerLayout = ...
    uigridlayout(plannerPanel,[1 1]);


plannerAxes = ...
    uiaxes(plannerLayout);


hold(plannerAxes,'on');


plannerAxes.Toolbar.Visible = ...
    'off';

plannerAxes.Color = ...
    [0.94 0.97 1];


xlim(plannerAxes,[0 100]);

ylim(plannerAxes,[-14 14]);


xlabel( ...
    plannerAxes, ...
    'Forward Distance');


ylabel( ...
    plannerAxes, ...
    'Road Width');


title( ...
    plannerAxes, ...
    'Top-Down Adaptive Driving View', ...
    'FontWeight','bold');


grid(plannerAxes,'on');


%% =========================================================
% ROAD
%% =========================================================

rectangle( ...
    plannerAxes, ...
    'Position', ...
    [0 -6 100 12], ...
    'FaceColor', ...
    [0.30 0.33 0.38], ...
    'EdgeColor', ...
    [0.55 0.62 0.70]);


%% Road boundaries

plot( ...
    plannerAxes, ...
    [0 100], ...
    [-6 -6], ...
    'LineWidth',2);


plot( ...
    plannerAxes, ...
    [0 100], ...
    [6 6], ...
    'LineWidth',2);


%% Broken centre guide

for x = 0:10:90

    plot( ...
        plannerAxes, ...
        [x x+5], ...
        [0 0], ...
        'LineWidth',1);

end


%% =========================================================
% REALISTIC EGO CAR
%% =========================================================

egoCar = ...
    createTopDownCar( ...
    plannerAxes, ...
    egoX, ...
    egoY, ...
    0, ...
    [0.05 0.52 0.95]);


%% =========================================================
% DETECTED VEHICLE FOR PLANNER
%% =========================================================

detectedCar = ...
    createTopDownCar( ...
    plannerAxes, ...
    60, ...
    0, ...
    0, ...
    [0.90 0.16 0.16]);


setCarVisible( ...
    detectedCar, ...
    'off');


%% =========================================================
% PEDESTRIAN MARKER
%% =========================================================

personMarker = ...
    scatter( ...
    plannerAxes, ...
    60, ...
    0, ...
    170, ...
    'filled', ...
    'Visible','off');


personLabel = ...
    text( ...
    plannerAxes, ...
    60, ...
    2, ...
    'PEDESTRIAN', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold', ...
    'Visible','off');


%% =========================================================
% SAFE PATH
%% =========================================================

pathPlot = ...
    plot( ...
    plannerAxes, ...
    linspace(egoX+3,95,100), ...
    zeros(1,100), ...
    'LineWidth',4);


%% Predicted motion

predictionPlot = ...
    plot( ...
    plannerAxes, ...
    nan, ...
    nan, ...
    '--', ...
    'LineWidth',2);


%% =========================================================
% STATUS PANEL
%% =========================================================

statusPanel = ...
    uipanel(mainGrid);

statusPanel.Layout.Row = 4;

statusPanel.Layout.Column = [1 2];

statusPanel.Title = ...
    'REAL-TIME ANALYTICS & STATUS';

statusPanel.FontWeight = ...
    'bold';

statusPanel.ForegroundColor = ...
    [0.02 0.40 0.85];

statusPanel.BackgroundColor = ...
    [1 1 1];


statusGrid = ...
    uigridlayout(statusPanel,[2 8]);

statusGrid.RowHeight = ...
    {30,50};

statusGrid.ColumnWidth = ...
    repmat({'1x'},1,8);


headers = { ...
    'OBJECT', ...
    'RISK LEVEL', ...
    'DECISION', ...
    'SPEED', ...
    'REPLAN LATENCY', ...
    'COLLISION', ...
    'PATH SMOOTHNESS', ...
    'SIM PROGRESS'};


for i = 1:8

    label = ...
        uilabel(statusGrid);

    label.Layout.Row = 1;

    label.Layout.Column = i;

    label.Text = ...
        headers{i};

    label.HorizontalAlignment = ...
        'center';

    label.FontWeight = ...
        'bold';

    label.FontColor = ...
        [0.35 0.45 0.60];

end


objectValue = ...
    createValue( ...
    statusGrid,1,'NONE');


riskValue = ...
    createValue( ...
    statusGrid,2,'SAFE');


decisionValue = ...
    createValue( ...
    statusGrid,3,'GO STRAIGHT');


speedValue = ...
    createValue( ...
    statusGrid,4,'30 km/h');


latencyValue = ...
    createValue( ...
    statusGrid,5,'-- ms');


collisionValue = ...
    createValue( ...
    statusGrid,6,'NO');


smoothnessValue = ...
    createValue( ...
    statusGrid,7,'--');


progressValue = ...
    createValue( ...
    statusGrid,8,'--');


%% =========================================================
% FOOTER
%% =========================================================

footerPanel = ...
    uipanel(mainGrid);

footerPanel.Layout.Row = 5;

footerPanel.Layout.Column = [1 2];

footerPanel.BackgroundColor = ...
    [0.985 0.99 1];


footerGrid = ...
    uigridlayout(footerPanel,[1 5]);

footerGrid.ColumnWidth = ...
    {'1.3x','1.1x','1x','1x','1.2x'};


scenarioLabel = ...
    createFooter( ...
    footerGrid,1, ...
    'Scenario: Live Camera');


inputLabel = ...
    createFooter( ...
    footerGrid,2, ...
    'Input: Not Selected');


trafficLabel = ...
    createFooter( ...
    footerGrid,3, ...
    'Traffic: --');


statusLabel = ...
    createFooter( ...
    footerGrid,4, ...
    'System: READY');


systemLabel = ...
    createFooter( ...
    footerGrid,5, ...
    'SYSTEM STATUS: READY');


%% =========================================================
% BUTTON CALLBACKS
%% =========================================================

liveButton.ButtonPushedFcn = ...
    @selectCamera;


videoButton.ButtonPushedFcn = ...
    @selectVideo;


scenarioButton.ButtonPushedFcn = ...
    @scenarioSelected;


startButton.ButtonPushedFcn = ...
    @startSystem;


stopButton.ButtonPushedFcn = ...
    @stopSystem;


resetButton.ButtonPushedFcn = ...
    @resetSystem;


%% =========================================================
% SELECT CAMERA
%% =========================================================

    function selectCamera(~,~)

        stopSystem();

        try

            if isempty(cam)

                cam = webcam(1);

            end


            frame = snapshot(cam);


            updateCameraDisplay(frame);


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
% SELECT VIDEO
%% =========================================================

    function selectVideo(~,~)

        stopSystem();


        [file,path] = ...
            uigetfile( ...
            {'*.mp4;*.mov;*.avi', ...
            'Road Videos'}, ...
            'Select Indian Road Video');


        if isequal(file,0)

            return;

        end


        try

            videoReader = ...
                VideoReader( ...
                fullfile(path,file));


            frame = ...
                readFrame(videoReader);


            updateCameraDisplay(frame);


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
% SCENARIO BUTTON
%% =========================================================

    function scenarioSelected(~,~)

        selected = ...
            string( ...
            scenarioDrop.Value);


        scenarioLabel.Text = ...
            "Scenario: " + ...
            selected;


        switch selected

            case "Dense Market Area"

                trafficLabel.Text = ...
                    'Traffic: HIGH';

            case ...
                    "Urban Unsignalised Intersection"

                trafficLabel.Text = ...
                    'Traffic: IRREGULAR';

            case ...
                    "Highway Slow-Vehicle Merge"

                trafficLabel.Text = ...
                    'Traffic: MERGING';

            case ...
                    "Sudden Cattle Crossing"

                trafficLabel.Text = ...
                    'Traffic: ANIMAL RISK';

            case ...
                    "Unmarked Village Road"

                trafficLabel.Text = ...
                    'Traffic: MIXED';

            otherwise

                trafficLabel.Text = ...
                    'Traffic: LIVE';

        end

    end


%% =========================================================
% START SYSTEM
%% =========================================================

    function startSystem(~,~)

        if running

            return;

        end


        if inputMode == "NONE"

            uialert( ...
                app, ...
                ['Choose LIVE CAMERA or ' ...
                'LOAD ROAD VIDEO first.'], ...
                'Input Required');

            return;

        end


        %% Load YOLO only once

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


        %% Timer

        if isempty(timerObj) || ...
                ~isvalid(timerObj)

            timerObj = ...
                timer( ...
                'ExecutionMode', ...
                'fixedSpacing', ...
                'Period', ...
                0.20, ...
                'BusyMode', ...
                'drop', ...
                'TimerFcn', ...
                @processFrame);

        end


        start(timerObj);

    end


%% =========================================================
% STOP SYSTEM
%% =========================================================

    function stopSystem(varargin)

        running = false;


        if ~isempty(timerObj)

            try

                if isvalid(timerObj) && ...
                        strcmp( ...
                        timerObj.Running, ...
                        'on')

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


        avoidanceMode = ...
            "NONE";


        avoidanceCounter = 0;


        updateTopDownCar( ...
            egoCar, ...
            egoX, ...
            egoY, ...
            0);


        setCarVisible( ...
            detectedCar, ...
            'off');


        personMarker.Visible = ...
            'off';


        personLabel.Visible = ...
            'off';


        predictionPlot.XData = ...
            nan;


        predictionPlot.YData = ...
            nan;


        pathPlot.XData = ...
            linspace( ...
            egoX+3,95,100);


        pathPlot.YData = ...
            zeros(1,100);


        objectValue.Text = ...
            'NONE';


        riskValue.Text = ...
            'SAFE';


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


        progressValue.Text = ...
            '--';


        leftCard.Text = ...
            'Waiting for AI...';


        centerCard.Text = ...
            'Waiting for AI...';


        rightCard.Text = ...
            'Waiting for AI...';


        statusLabel.Text = ...
            'System: READY';


        systemLabel.Text = ...
            'SYSTEM STATUS: READY';

    end


%% =========================================================
% PROCESS EACH FRAME
%% =========================================================

    function processFrame(~,~)

        if ~running || ...
                ~isvalid(app)

            return;

        end


        %% =================================================
        % GET INPUT FRAME
        %% =================================================

        try

            if inputMode == "CAMERA"

                frame = ...
                    snapshot(cam);


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


        %% =================================================
        % YOLO
        %% =================================================

        try

            [bboxes,scores,labels] = ...
                detect( ...
                detector, ...
                frame, ...
                Threshold=0.35);

        catch

            return;

        end


        labels = ...
            string(labels);


        %% Keep relevant classes

        classes = [ ...
            "person", ...
            "car", ...
            "motorcycle", ...
            "bicycle", ...
            "bus", ...
            "truck"];


        keep = ...
            ismember( ...
            labels, ...
            classes);


        bboxes = ...
            bboxes(keep,:);


        scores = ...
            scores(keep);


        labels = ...
            labels(keep);


        %% =================================================
        % FREE SPACE ANALYSIS
        %% =================================================

        third = frameW/3;


        leftOcc = 0;

        centerOcc = 0;

        rightOcc = 0;


        vehicleThreat = false;

        vulnerableThreat = false;

        criticalThreat = false;


        primaryObject = ...
            "NONE";


        highestScore = 0;


        %% Analyse objects

        for i = 1:size(bboxes,1)


            box = ...
                bboxes(i,:);


            x = box(1);

            y = box(2);

            w = box(3);

            h = box(4);


            objectCenterX = ...
                x + w/2;


            bottomY = ...
                y + h;


            areaRatio = ...
                (w*h) / ...
                (frameW*frameH);


            bottomRatio = ...
                bottomY / ...
                frameH;


            %% Primary object

            if scores(i) > ...
                    highestScore

                highestScore = ...
                    scores(i);


                primaryObject = ...
                    upper(labels(i));

            end


            %% Occupancy

            if objectCenterX < third

                leftOcc = ...
                    leftOcc + ...
                    areaRatio;


            elseif objectCenterX < ...
                    2*third

                centerOcc = ...
                    centerOcc + ...
                    areaRatio;

            else

                rightOcc = ...
                    rightOcc + ...
                    areaRatio;

            end


            %% Center path

            inCenter = ...
                objectCenterX > third && ...
                objectCenterX < 2*third;


            %% Object type

            vulnerable = ...
                labels(i) == "person" || ...
                labels(i) == "bicycle" || ...
                labels(i) == "motorcycle";


            vehicleObject = ...
                labels(i) == "car" || ...
                labels(i) == "bus" || ...
                labels(i) == "truck";


            %% Proximity estimate

            criticalObject = ...
                areaRatio > 0.14 && ...
                bottomRatio > 0.78;


            closeObject = ...
                areaRatio > 0.05 && ...
                bottomRatio > 0.58;


            mediumObject = ...
                areaRatio > 0.012 && ...
                bottomRatio > 0.42;


            %% Threat

            if inCenter


                if vehicleObject && ...
                        (closeObject || ...
                         mediumObject)

                    vehicleThreat = ...
                        true;

                end


                if vulnerable && ...
                        (closeObject || ...
                         mediumObject)

                    vulnerableThreat = ...
                        true;

                end


                if vulnerable && ...
                        criticalObject

                    criticalThreat = ...
                        true;

                end

            end

        end


        %% Free scores

        leftFree = ...
            max(0,1-leftOcc);


        centerFree = ...
            max(0,1-centerOcc);


        rightFree = ...
            max(0,1-rightOcc);


        %% =================================================
        % REPLANNING TIMER
        %% =================================================

        planningTimer = tic;


        %% =================================================
        % DECISION LOGIC
        %% =================================================

        risk = "SAFE";

        decision = ...
            "GO STRAIGHT";


        %% Critical pedestrian/bike

        if criticalThreat


            risk = ...
                "CRITICAL";


            decision = ...
                "BRAKE";


            avoidanceMode = ...
                "NONE";


            avoidanceCounter = 0;


        %% Continue active avoidance

        elseif avoidanceMode ~= "NONE"


            risk = ...
                "HIGH";


            decision = ...
                avoidanceMode;


            avoidanceCounter = ...
                avoidanceCounter - 1;


            if avoidanceCounter <= 0

                avoidanceMode = ...
                    "NONE";

            end


        %% Vehicle ahead

        elseif vehicleThreat


            risk = ...
                "HIGH";


            leftClear = ...
                leftFree > 0.90;


            rightClear = ...
                rightFree > 0.90;


            if leftClear && ...
                    ~rightClear

                decision = ...
                    "MOVE LEFT";


            elseif rightClear && ...
                    ~leftClear

                decision = ...
                    "MOVE RIGHT";


            elseif leftClear && ...
                    rightClear


                if leftFree >= ...
                        rightFree

                    decision = ...
                        "MOVE LEFT";

                else

                    decision = ...
                        "MOVE RIGHT";

                end

            else

                decision = ...
                    "BRAKE";

            end


            if decision ~= "BRAKE"

                avoidanceMode = ...
                    decision;


                avoidanceCounter = ...
                    avoidanceDuration;

            end


        %% Person/bike moderate risk

        elseif vulnerableThreat


            risk = ...
                "MEDIUM";


            decision = ...
                "SLOW";

        end


        %% =================================================
        % VEHICLE MOVEMENT
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


        %% Smooth steering

        lateralError = ...
            targetY - egoY;


        sideStep = max( ...
            min( ...
            lateralError, ...
            0.22), ...
            -0.22);


        egoY = ...
            egoY + ...
            sideStep;


        egoX = ...
            egoX + ...
            forwardStep;


        %% Vehicle heading

        if forwardStep > 0

            heading = ...
                atan2( ...
                sideStep, ...
                forwardStep);

        else

            heading = 0;

        end


        %% Loop road

        if egoX > 91

            egoX = 8;

            egoY = 0;

            avoidanceMode = ...
                "NONE";

            avoidanceCounter = 0;

        end


        %% Update realistic car

        updateTopDownCar( ...
            egoCar, ...
            egoX, ...
            egoY, ...
            heading);


        %% =================================================
        % DISPLAY DETECTED OBJECT
        %% =================================================

        obstacleX = ...
            min(84,egoX+25);


        if primaryObject == "CAR" || ...
           primaryObject == "BUS" || ...
           primaryObject == "TRUCK"


            setCarVisible( ...
                detectedCar, ...
                'on');


            updateTopDownCar( ...
                detectedCar, ...
                obstacleX, ...
                0, ...
                0);


            personMarker.Visible = ...
                'off';


            personLabel.Visible = ...
                'off';


            predictionPlot.XData = ...
                [obstacleX ...
                 obstacleX+8];


            predictionPlot.YData = ...
                [0 0];


        elseif primaryObject == "PERSON" || ...
               primaryObject == "BICYCLE" || ...
               primaryObject == "MOTORCYCLE"


            setCarVisible( ...
                detectedCar, ...
                'off');


            personMarker.Visible = ...
                'on';


            personMarker.XData = ...
                obstacleX;


            personMarker.YData = ...
                0;


            personLabel.Visible = ...
                'on';


            personLabel.Position = ...
                [obstacleX 2 0];


            personLabel.String = ...
                primaryObject;


            predictionPlot.XData = ...
                [obstacleX ...
                 obstacleX];


            predictionPlot.YData = ...
                [0 -4];


        else


            setCarVisible( ...
                detectedCar, ...
                'off');


            personMarker.Visible = ...
                'off';


            personLabel.Visible = ...
                'off';


            predictionPlot.XData = ...
                nan;


            predictionPlot.YData = ...
                nan;

        end


        %% =================================================
        % SAFE PATH
        %% =================================================

        pathEnd = ...
            min(98,egoX+48);


        px = linspace( ...
            egoX+3, ...
            pathEnd, ...
            120);


        switch decision


            case "MOVE LEFT"

                py = ...
                    makeAvoidancePath( ...
                    px, ...
                    egoY, ...
                    4);


            case "MOVE RIGHT"

                py = ...
                    makeAvoidancePath( ...
                    px, ...
                    egoY, ...
                    -4);


            case "BRAKE"

                px = ...
                    linspace( ...
                    egoX+3, ...
                    min( ...
                    98, ...
                    egoX+10), ...
                    35);


                py = ...
                    egoY * ...
                    ones(size(px));


            otherwise

                py = ...
                    makeReturnPath( ...
                    px, ...
                    egoY);

        end


        pathPlot.XData = px;

        pathPlot.YData = py;


        %% Replanning latency

        replanningLatency = ...
            toc(planningTimer)*1000;


        %% =================================================
        % PATH SMOOTHNESS
        %% =================================================

        if length(py) > 2

            roughness = ...
                mean( ...
                abs( ...
                diff(py,2)));


            smoothness = ...
                max( ...
                0, ...
                min( ...
                1, ...
                1-roughness));

        else

            smoothness = 1;

        end


        %% =================================================
        % DRAW YOLO
        %% =================================================

        if ~isempty(bboxes)


            detectorText = ...
                strings( ...
                length(labels),1);


            for i = 1:length(labels)


                detectorText(i) = ...
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
                cellstr( ...
                detectorText), ...
                'LineWidth',3);

        else

            output = frame;

        end


        %% =================================================
        % LEFT / CENTER / RIGHT BOXES
        %% =================================================

        zoneTop = ...
            round(frameH*0.30);


        zoneHeight = ...
            round(frameH*0.65);


        leftRectangle = [ ...
            1 ...
            zoneTop ...
            round(third) ...
            zoneHeight];


        centerRectangle = [ ...
            round(third)+1 ...
            zoneTop ...
            round(third) ...
            zoneHeight];


        rightRectangle = [ ...
            round(2*third)+1 ...
            zoneTop ...
            frameW-round(2*third) ...
            zoneHeight];


        output = ...
            insertShape( ...
            output, ...
            "Rectangle", ...
            leftRectangle, ...
            'LineWidth',2);


        output = ...
            insertShape( ...
            output, ...
            "Rectangle", ...
            centerRectangle, ...
            'LineWidth',3);


        output = ...
            insertShape( ...
            output, ...
            "Rectangle", ...
            rightRectangle, ...
            'LineWidth',2);


        %% Free space labels

        output = ...
            insertText( ...
            output, ...
            [10 zoneTop+8], ...
            sprintf( ...
            "LEFT\nFREE %.0f%%", ...
            leftFree*100), ...
            'FontSize',14, ...
            'BoxOpacity',0.60);


        output = ...
            insertText( ...
            output, ...
            [third+10 zoneTop+8], ...
            sprintf( ...
            "CENTER\nFREE %.0f%%", ...
            centerFree*100), ...
            'FontSize',14, ...
            'BoxOpacity',0.60);


        output = ...
            insertText( ...
            output, ...
            [2*third+10 zoneTop+8], ...
            sprintf( ...
            "RIGHT\nFREE %.0f%%", ...
            rightFree*100), ...
            'FontSize',14, ...
            'BoxOpacity',0.60);


        %% =================================================
        % UPDATE CAMERA
        %% =================================================

        updateCameraDisplay(output);


        %% =================================================
        % STATUS
        %% =================================================

        objectValue.Text = ...
            primaryObject;


        riskValue.Text = ...
            risk;


        %% Risk color

        if risk == "SAFE"

            riskValue.FontColor = ...
                [0 0.62 0.42];

        elseif risk == "MEDIUM"

            riskValue.FontColor = ...
                [0.92 0.50 0];

        else

            riskValue.FontColor = ...
                [0.88 0.12 0.15];

        end


        decisionValue.Text = ...
            decision;


        speedValue.Text = ...
            string(speed) + ...
            " km/h";


        latencyValue.Text = ...
            sprintf( ...
            '%.1f ms', ...
            replanningLatency);


        collisionValue.Text = ...
            'NO';


        smoothnessValue.Text = ...
            sprintf( ...
            '%.2f', ...
            smoothness);


        progress = ...
            min( ...
            100, ...
            max( ...
            0, ...
            egoX/91*100));


        progressValue.Text = ...
            sprintf( ...
            '%.0f%%', ...
            progress);


        %% Analysis cards

        leftCard.Text = ...
            sprintf( ...
            ['Free space %.0f%%\n' ...
             '%s'], ...
            leftFree*100, ...
            getSideStatus( ...
            leftFree));


        centerCard.Text = ...
            sprintf( ...
            ['Forward free %.0f%%\n' ...
             '%s'], ...
            centerFree*100, ...
            getCenterStatus( ...
            risk));


        rightCard.Text = ...
            sprintf( ...
            ['Free space %.0f%%\n' ...
             '%s'], ...
            rightFree*100, ...
            getSideStatus( ...
            rightFree));


        %% Traffic level

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
% UPDATE CAMERA IMAGE
%% =========================================================

    function updateCameraDisplay(frame)

        cameraImage.CData = frame;


        cameraImage.XData = ...
            [1 size(frame,2)];


        cameraImage.YData = ...
            [1 size(frame,1)];


        xlim( ...
            cameraAxes, ...
            [1 size(frame,2)]);


        ylim( ...
            cameraAxes, ...
            [1 size(frame,1)]);


        cameraAxes.YDir = ...
            'reverse';


        axis( ...
            cameraAxes, ...
            'image');

    end


%% =========================================================
% CLOSE APP
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
% CREATE ANALYSIS CARD
%% =========================================================

function value = createAnalysisCard( ...
    parent,column,titleText)


panel = ...
    uipanel(parent);


panel.Layout.Column = ...
    column;


panel.BackgroundColor = ...
    [0.97 0.985 1];


grid = ...
    uigridlayout( ...
    panel,[2 1]);


grid.RowHeight = ...
    {30,'1x'};


header = ...
    uilabel(grid);


header.Text = ...
    titleText;


header.FontWeight = ...
    'bold';


header.FontColor = ...
    [0.02 0.40 0.82];


value = ...
    uilabel(grid);


value.Text = ...
    'Waiting for AI...';


value.FontColor = ...
    [0.25 0.38 0.52];


value.VerticalAlignment = ...
    'top';

end


%% =========================================================
% STATUS VALUE
%% =========================================================

function h = createValue( ...
    parent,column,textValue)


h = ...
    uilabel(parent);


h.Layout.Row = 2;


h.Layout.Column = ...
    column;


h.Text = ...
    textValue;


h.HorizontalAlignment = ...
    'center';


h.FontWeight = ...
    'bold';


h.FontSize = 14;


h.FontColor = ...
    [0.08 0.28 0.50];

end


%% =========================================================
% FOOTER
%% =========================================================

function h = createFooter( ...
    parent,column,textValue)


h = ...
    uilabel(parent);


h.Layout.Column = ...
    column;


h.Text = ...
    textValue;


h.FontWeight = ...
    'bold';


h.FontColor = ...
    [0.20 0.36 0.52];

end


%% =========================================================
% CREATE REALISTIC TOP-DOWN CAR
%% =========================================================

function car = createTopDownCar( ...
    ax,x,y,heading,bodyColor)


%% Main silhouette

body = [ ...

    -3.0 -0.70
    -2.70 -0.95
    -2.10 -1.10

     1.75 -1.10
     2.40 -0.90
     2.85 -0.55

     3.05  0

     2.85  0.55
     2.40  0.90
     1.75  1.10

    -2.10  1.10
    -2.70  0.95
    -3.0   0.70

];


%% Passenger cabin

cabin = [ ...

    -1.35 -0.72
     0.85 -0.72
     1.45 -0.48
     1.55  0
     1.45  0.48
     0.85  0.72
    -1.35  0.72
    -1.60  0.42
    -1.60 -0.42

];


%% Front glass

frontGlass = [ ...

    0.70 -0.64
    1.35 -0.42
    1.45  0
    1.35  0.42
    0.70  0.64

];


%% Rear glass

rearGlass = [ ...

    -1.42 -0.55
    -0.95 -0.68
    -0.95  0.68
    -1.42  0.55

];


%% Wheels

wheelFL = ...
    makeWheel( ...
    1.65,-1.15);


wheelFR = ...
    makeWheel( ...
    1.65,1.15);


wheelRL = ...
    makeWheel( ...
    -1.65,-1.15);


wheelRR = ...
    makeWheel( ...
    -1.65,1.15);


%% Save local geometry

car.bodyShape = body;

car.cabinShape = cabin;

car.frontGlassShape = ...
    frontGlass;

car.rearGlassShape = ...
    rearGlass;

car.wheelFLShape = ...
    wheelFL;

car.wheelFRShape = ...
    wheelFR;

car.wheelRLShape = ...
    wheelRL;

car.wheelRRShape = ...
    wheelRR;


%% Body

p = ...
    transformPoints( ...
    body,x,y,heading);


car.body = ...
    patch( ...
    ax, ...
    p(:,1), ...
    p(:,2), ...
    bodyColor, ...
    'EdgeColor', ...
    [0.02 0.22 0.50], ...
    'LineWidth',1.8);


%% Cabin

p = ...
    transformPoints( ...
    cabin,x,y,heading);


car.cabin = ...
    patch( ...
    ax, ...
    p(:,1), ...
    p(:,2), ...
    [0.08 0.18 0.30], ...
    'EdgeColor', ...
    [0.60 0.82 0.98]);


%% Front windshield

p = ...
    transformPoints( ...
    frontGlass,x,y,heading);


car.frontGlass = ...
    patch( ...
    ax, ...
    p(:,1), ...
    p(:,2), ...
    [0.45 0.76 0.92], ...
    'EdgeColor','none');


%% Rear windshield

p = ...
    transformPoints( ...
    rearGlass,x,y,heading);


car.rearGlass = ...
    patch( ...
    ax, ...
    p(:,1), ...
    p(:,2), ...
    [0.30 0.58 0.78], ...
    'EdgeColor','none');


%% Wheels

car.wheelFL = ...
    createWheelPatch( ...
    ax, ...
    wheelFL, ...
    x,y,heading);


car.wheelFR = ...
    createWheelPatch( ...
    ax, ...
    wheelFR, ...
    x,y,heading);


car.wheelRL = ...
    createWheelPatch( ...
    ax, ...
    wheelRL, ...
    x,y,heading);


car.wheelRR = ...
    createWheelPatch( ...
    ax, ...
    wheelRR, ...
    x,y,heading);


%% Label

car.label = ...
    text( ...
    ax, ...
    x, ...
    y-2.0, ...
    'EGO', ...
    'HorizontalAlignment', ...
    'center', ...
    'FontWeight','bold', ...
    'Color', ...
    [0.03 0.35 0.75]);

end


%% =========================================================
% UPDATE TOP-DOWN CAR
%% =========================================================

function updateTopDownCar( ...
    car,x,y,heading)


updatePatch( ...
    car.body, ...
    car.bodyShape, ...
    x,y,heading);


updatePatch( ...
    car.cabin, ...
    car.cabinShape, ...
    x,y,heading);


updatePatch( ...
    car.frontGlass, ...
    car.frontGlassShape, ...
    x,y,heading);


updatePatch( ...
    car.rearGlass, ...
    car.rearGlassShape, ...
    x,y,heading);


updatePatch( ...
    car.wheelFL, ...
    car.wheelFLShape, ...
    x,y,heading);


updatePatch( ...
    car.wheelFR, ...
    car.wheelFRShape, ...
    x,y,heading);


updatePatch( ...
    car.wheelRL, ...
    car.wheelRLShape, ...
    x,y,heading);


updatePatch( ...
    car.wheelRR, ...
    car.wheelRRShape, ...
    x,y,heading);


car.label.Position = ...
    [x y-2.0 0];

end


%% =========================================================
% SET CAR VISIBILITY
%% =========================================================

function setCarVisible( ...
    car,state)


car.body.Visible = state;

car.cabin.Visible = state;

car.frontGlass.Visible = state;

car.rearGlass.Visible = state;

car.wheelFL.Visible = state;

car.wheelFR.Visible = state;

car.wheelRL.Visible = state;

car.wheelRR.Visible = state;

car.label.Visible = state;

end


%% =========================================================
% UPDATE PATCH
%% =========================================================

function updatePatch( ...
    handle,shape,x,y,heading)


p = ...
    transformPoints( ...
    shape,x,y,heading);


handle.XData = ...
    p(:,1);


handle.YData = ...
    p(:,2);

end


%% =========================================================
% CREATE WHEEL PATCH
%% =========================================================

function h = createWheelPatch( ...
    ax,shape,x,y,heading)


p = ...
    transformPoints( ...
    shape,x,y,heading);


h = ...
    patch( ...
    ax, ...
    p(:,1), ...
    p(:,2), ...
    [0.04 0.04 0.04], ...
    'EdgeColor','none');

end


%% =========================================================
% WHEEL GEOMETRY
%% =========================================================

function wheel = makeWheel(cx,cy)


L = 0.85;

W = 0.28;


wheel = [ ...

    cx-L/2 cy-W/2
    cx+L/2 cy-W/2
    cx+L/2 cy+W/2
    cx-L/2 cy+W/2

];

end


%% =========================================================
% TRANSFORM POINTS
%% =========================================================

function p = transformPoints( ...
    local,x,y,heading)


R = [ ...

    cos(heading) ...
    -sin(heading)

    sin(heading) ...
     cos(heading)

];


p = ...
    local * R';


p(:,1) = ...
    p(:,1) + x;


p(:,2) = ...
    p(:,2) + y;

end


%% =========================================================
% SMOOTH STEP
%% =========================================================

function y = smoothStep(t)


y = ...
    t.^2 .* ...
    (3 - 2*t);

end


%% =========================================================
% AVOIDANCE PATH
%% =========================================================

function py = makeAvoidancePath( ...
    px,currentY,targetY)


n = length(px);


py = ...
    zeros(size(px));


for k = 1:n


    t = ...
        (k-1)/(n-1);


    if t < 0.35


        u = ...
            t/0.35;


        py(k) = ...
            currentY + ...
            (targetY-currentY) * ...
            smoothStep(u);


    elseif t < 0.70


        py(k) = ...
            targetY;


    else


        u = ...
            (t-0.70)/0.30;


        py(k) = ...
            targetY * ...
            (1-smoothStep(u));

    end

end

end


%% =========================================================
% RETURN-TO-CENTER PATH
%% =========================================================

function py = makeReturnPath( ...
    px,currentY)


n = length(px);


py = ...
    zeros(size(px));


for k = 1:n


    t = ...
        (k-1)/(n-1);


    py(k) = ...
        currentY * ...
        (1-smoothStep(t));

end

end


%% =========================================================
% ANALYSIS TEXT
%% =========================================================

function textValue = ...
    getSideStatus(freeValue)


if freeValue > 0.93

    textValue = ...
        'Safe side candidate';

elseif freeValue > 0.80

    textValue = ...
        'Moderate clearance';

else

    textValue = ...
        'Side occupied';

end

end


function textValue = ...
    getCenterStatus(risk)


if risk == "SAFE"

    textValue = ...
        'Forward path clear';

elseif risk == "MEDIUM"

    textValue = ...
        'Caution / slowing';

elseif risk == "HIGH"

    textValue = ...
        'Obstacle / replanning';

else

    textValue = ...
        'Emergency hazard';

end

end