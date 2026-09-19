clear;
clc;
close all;

%% PHASE 7 - LIVE CAMERA + YOLO

disp("Loading YOLO...");
detector = yolov4ObjectDetector("tiny-yolov4-coco");

%% Connect camera
cam = webcam(1);

%% Display window
fig = figure( ...
    'Name','Live AI Road Perception', ...
    'NumberTitle','off');

while ishandle(fig)

    %% Capture frame
    frame = snapshot(cam);

    %% Detect objects
    [bboxes,scores,labels] = detect( ...
        detector, ...
        frame, ...
        Threshold=0.35);

    %% Keep road-related objects only
    wanted = ...
        labels == "person" | ...
        labels == "car" | ...
        labels == "motorcycle" | ...
        labels == "bus" | ...
        labels == "truck" | ...
        labels == "bicycle";

    bboxes = bboxes(wanted,:);
    scores = scores(wanted);
    labels = labels(wanted);

    %% Make text labels
    textLabels = strings(length(labels),1);

    for i = 1:length(labels)

        textLabels(i) = ...
            upper(string(labels(i))) + ...
            " " + ...
            string(round(scores(i)*100)) + "%";

    end

    %% Draw boxes
    if ~isempty(bboxes)

        output = insertObjectAnnotation( ...
            frame, ...
            "rectangle", ...
            bboxes, ...
            textLabels, ...
            'LineWidth',3);

    else

        output = frame;

    end

    %% Show output
    imshow(output);

    title(sprintf( ...
        'LIVE YOLO ROAD PERCEPTION | Objects: %d', ...
        length(labels)), ...
        'FontSize',14, ...
        'FontWeight','bold');

    drawnow;

end

%% Release camera
clear cam;