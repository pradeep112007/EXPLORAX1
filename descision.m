%% =====================================================
% IMPROVED DECISION SYSTEM
%% =====================================================

risk = "SAFE";
decision = "GO STRAIGHT";

%% -----------------------------------------------------
% PRIORITY 1 - VERY CLOSE PERSON / BIKE
%% -----------------------------------------------------

if criticalPedestrianThreat

    avoidanceMode = "NONE";
    avoidanceCounter = 0;

    risk = "CRITICAL";
    decision = "BRAKE";

    %% -----------------------------------------------------
    % PRIORITY 2 - CONTINUE CURRENT AVOIDANCE
    %% -----------------------------------------------------

elseif avoidanceMode ~= "NONE"

    risk = "HIGH";
    decision = avoidanceMode;

    avoidanceCounter = avoidanceCounter - 1;

    if avoidanceCounter <= 0
        avoidanceMode = "NONE";
    end

    %% -----------------------------------------------------
    % PRIORITY 3 - CAR / BUS / TRUCK AHEAD
    %% -----------------------------------------------------

elseif vehicleThreat

    risk = "HIGH";

    % Pick the freer side
    if leftFree > rightFree + 0.02

        decision = "MOVE LEFT";

    elseif rightFree > leftFree + 0.02

        decision = "MOVE RIGHT";

    else

        % Both sides similar
        decision = "MOVE LEFT";

    end

    avoidanceMode = decision;
    avoidanceCounter = avoidanceDuration;

    %% -----------------------------------------------------
    % PRIORITY 4 - PERSON / BIKE NOT EXTREMELY CLOSE
    %% -----------------------------------------------------

elseif vulnerableThreat

    risk = "MEDIUM";
    decision = "SLOW";

    %% -----------------------------------------------------
    % PRIORITY 5 - OTHER MEDIUM RISK
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