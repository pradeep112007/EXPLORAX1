function final_dashboard

clc;

%% =========================================================
% MAIN WINDOW
%% =========================================================

app = uifigure( ...
    'Name','Adaptive Path Planning - Indian Roads', ...
    'Position',[100 80 1450 820], ...
    'Color',[0.07 0.08 0.11]);

%% =========================================================
% MAIN GRID
%% =========================================================

mainGrid = uigridlayout(app,[5 2]);

mainGrid.RowHeight = {
    70, ...
    60, ...
    '1x', ...
    100, ...
    55
};

mainGrid.ColumnWidth = {
    '1x', ...
    '1x'
};

mainGrid.Padding = [15 15 15 15];

mainGrid.RowSpacing = 12;
mainGrid.ColumnSpacing = 12;

%% =========================================================
% TITLE
%% =========================================================

titleLabel = uilabel(mainGrid);

titleLabel.Layout.Row = 1;
titleLabel.Layout.Column = [1 2];

titleLabel.Text = ...
    'ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE';

titleLabel.FontSize = 25;
titleLabel.FontWeight = 'bold';

titleLabel.FontColor = [0.3 0.8 1];

titleLabel.HorizontalAlignment = 'center';

%% =========================================================
% BUTTON PANEL
%% =========================================================

controlPanel = uipanel(mainGrid);

controlPanel.Layout.Row = 2;
controlPanel.Layout.Column = [1 2];

controlPanel.BackgroundColor = [0.10 0.11 0.15];

controls = uigridlayout(controlPanel,[1 6]);

controls.ColumnWidth = {
    '1x','1x','1x','1x','1x','1x'
};

%% LIVE CAMERA

liveButton = uibutton(controls,'push');

liveButton.Text = 'LIVE CAMERA';

liveButton.FontWeight = 'bold';

%% LOAD VIDEO

videoButton = uibutton(controls,'push');

videoButton.Text = 'LOAD ROAD VIDEO';

videoButton.FontWeight = 'bold';

%% SCENARIO 1

scenarioButton = uibutton(controls,'push');

scenarioButton.Text = 'SCENARIOS';

scenarioButton.FontWeight = 'bold';

%% START

startButton = uibutton(controls,'push');

startButton.Text = 'START AI';

startButton.FontWeight = 'bold';

%% STOP

stopButton = uibutton(controls,'push');

stopButton.Text = 'STOP';

stopButton.FontWeight = 'bold';

%% RESET

resetButton = uibutton(controls,'push');

resetButton.Text = 'RESET';

resetButton.FontWeight = 'bold';

%% =========================================================
% CAMERA PANEL
%% =========================================================

cameraPanel = uipanel(mainGrid);

cameraPanel.Layout.Row = 3;
cameraPanel.Layout.Column = 1;

cameraPanel.Title = 'LIVE PERCEPTION';

cameraPanel.FontWeight = 'bold';

cameraPanel.ForegroundColor = [0.9 0.9 0.9];

cameraPanel.BackgroundColor = [0.08 0.09 0.12];

cameraAxes = uiaxes(cameraPanel);

cameraAxes.Position = [15 15 670 430];

cameraAxes.Color = [0.03 0.03 0.04];

cameraAxes.XTick = [];
cameraAxes.YTick = [];

title(cameraAxes, ...
    'Camera / Road Video', ...
    'Color',[1 1 1]);

%% =========================================================
% PATH PLANNER PANEL
%% =========================================================

plannerPanel = uipanel(mainGrid);

plannerPanel.Layout.Row = 3;
plannerPanel.Layout.Column = 2;

plannerPanel.Title = 'ADAPTIVE PATH PLANNER';

plannerPanel.FontWeight = 'bold';

plannerPanel.ForegroundColor = [0.9 0.9 0.9];

plannerPanel.BackgroundColor = [0.08 0.09 0.12];

plannerAxes = uiaxes(plannerPanel);

plannerAxes.Position = [15 15 670 430];

hold(plannerAxes,'on');

plannerAxes.Color = [0.12 0.12 0.13];

plannerAxes.XColor = [0.8 0.8 0.8];

plannerAxes.YColor = [0.8 0.8 0.8];

xlim(plannerAxes,[0 100]);

ylim(plannerAxes,[-12 12]);

xlabel(plannerAxes,'Forward Distance');

ylabel(plannerAxes,'Road Width');

title(plannerAxes, ...
    'Top-Down Driving View', ...
    'Color',[1 1 1]);

%% =========================================================
% DRAW ROAD
%% =========================================================

rectangle( ...
    plannerAxes, ...
    'Position',[0 -6 100 12], ...
    'FaceColor',[0.18 0.18 0.18], ...
    'EdgeColor',[0.5 0.5 0.5]);

%% Ego vehicle

egoVehicle = rectangle( ...
    plannerAxes, ...
    'Position',[8 -1 6 2], ...
    'Curvature',0.25, ...
    'FaceColor',[0.15 0.55 1]);

%% Initial path

pathPlot = plot( ...
    plannerAxes, ...
    [14 95], ...
    [0 0], ...
    'LineWidth',4);

%% =========================================================
% STATUS PANEL
%% =========================================================

statusPanel = uipanel(mainGrid);

statusPanel.Layout.Row = 4;
statusPanel.Layout.Column = [1 2];

statusPanel.BackgroundColor = [0.10 0.11 0.15];

statusGrid = uigridlayout(statusPanel,[2 6]);

statusGrid.RowHeight = {35,35};

%% ---------------------------------------------------------
% OBJECT
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','OBJECT', ...
    'FontColor',[0.6 0.6 0.65]);

objectValue = uilabel(statusGrid);

objectValue.Text = 'NONE';

objectValue.FontWeight = 'bold';

objectValue.FontColor = [1 1 1];

%% ---------------------------------------------------------
% RISK
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','RISK LEVEL', ...
    'FontColor',[0.6 0.6 0.65]);

riskValue = uilabel(statusGrid);

riskValue.Text = 'SAFE';

riskValue.FontWeight = 'bold';

riskValue.FontColor = [0.2 1 0.4];

%% ---------------------------------------------------------
% DECISION
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','DECISION', ...
    'FontColor',[0.6 0.6 0.65]);

decisionValue = uilabel(statusGrid);

decisionValue.Text = 'GO STRAIGHT';

decisionValue.FontWeight = 'bold';

decisionValue.FontColor = [0.3 0.8 1];

%% ---------------------------------------------------------
% SPEED
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','SPEED', ...
    'FontColor',[0.6 0.6 0.65]);

speedValue = uilabel(statusGrid);

speedValue.Text = '30 km/h';

speedValue.FontWeight = 'bold';

speedValue.FontColor = [1 1 1];

%% ---------------------------------------------------------
% REPLANNING TIME
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','REPLAN LATENCY', ...
    'FontColor',[0.6 0.6 0.65]);

latencyValue = uilabel(statusGrid);

latencyValue.Text = '-- ms';

latencyValue.FontWeight = 'bold';

latencyValue.FontColor = [1 1 1];

%% ---------------------------------------------------------
% COLLISION
%% ---------------------------------------------------------

uilabel(statusGrid, ...
    'Text','COLLISION', ...
    'FontColor',[0.6 0.6 0.65]);

collisionValue = uilabel(statusGrid);

collisionValue.Text = 'NO';

collisionValue.FontWeight = 'bold';

collisionValue.FontColor = [0.2 1 0.4];

%% =========================================================
% BOTTOM STATUS
%% =========================================================

scenarioLabel = uilabel(mainGrid);

scenarioLabel.Layout.Row = 5;
scenarioLabel.Layout.Column = [1 2];

scenarioLabel.Text = ...
    'Scenario: Live Camera    |    System Status: READY';

scenarioLabel.HorizontalAlignment = 'center';

scenarioLabel.FontSize = 14;

scenarioLabel.FontWeight = 'bold';

scenarioLabel.FontColor = [0.75 0.75 0.8];

end