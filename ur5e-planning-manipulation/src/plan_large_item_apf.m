function plan = plan_large_item_apf(scene)
%PLAN_LARGE_ITEM_APF 使用中心力、角点排斥力和力矩规划大物体位姿。
%   输出path为N x 3 [x,y,theta]。安全诊断不会改变APF递推结果。

    [startPose, goalPose, obstacles, xLimits, yLimits] = ...
        large_item_scene_inputs(scene);
    config = large_item_planning_config();
    parameters = config.apf;
    geometry = config.geometry;

    currentPosition = startPose(1:2);
    currentTheta = startPose(3);
    path = [currentPosition, currentTheta];
    gap = norm(goalPose(1:2) - currentPosition);
    iteration = 0;
    lastDeltaPosition = [0, 0];
    lastDeltaTheta = 0;
    stoppedByZeroMotion = false;

    while gap > parameters.positionTolerance ...
            && iteration < parameters.maximumIterations
        currentPose = [currentPosition, currentTheta];
        corners = large_item_corners( ...
            currentPose, geometry.localCorners);

        centerForce = attractive_force( ...
            currentPosition, goalPose(1:2), parameters) ...
            + repulsive_force(currentPosition, obstacles, ...
                parameters.centerRepulsionDistance, ...
                parameters.centerRepulsiveGain, ...
                parameters.centerMaximumRepulsion);
        cornerForces = zeros(4, 2);
        for cornerIndex = 1:4
            cornerForces(cornerIndex, :) = repulsive_force( ...
                corners(cornerIndex, :), obstacles, ...
                parameters.cornerRepulsionDistance, ...
                parameters.cornerRepulsiveGain, ...
                parameters.cornerMaximumRepulsion);
        end
        totalForce = centerForce + mean(cornerForces, 1);

        arms = corners - currentPosition;
        repulsiveTorque = sum( ...
            arms(:, 1) .* cornerForces(:, 2) ...
            - arms(:, 2) .* cornerForces(:, 1));
        thetaWeight = max(0, min(1, ...
            1 - gap / parameters.thetaStartDistance));
        thetaError = goalPose(3) - currentTheta;
        limitedThetaError = max(-pi, min(pi, thetaError));
        goalTorque = thetaWeight * parameters.goalTorqueGain ...
            * limitedThetaError;
        totalTorque = repulsiveTorque + goalTorque;

        forceMagnitude = norm(totalForce);
        if forceMagnitude >= parameters.forceTolerance
            targetDeltaPosition = parameters.step ...
                * totalForce / forceMagnitude;
        else
            targetDeltaPosition = [0, 0];
        end
        deltaPosition = parameters.translationFilterWeight ...
            * lastDeltaPosition ...
            + (1 - parameters.translationFilterWeight) ...
            * targetDeltaPosition;
        lastDeltaPosition = deltaPosition;

        if abs(totalTorque) > parameters.torqueTolerance
            targetDeltaTheta = parameters.torqueToAngleGain ...
                * totalTorque;
        else
            targetDeltaTheta = 0;
        end
        targetDeltaTheta = max(-parameters.maximumThetaStep, ...
            min(parameters.maximumThetaStep, targetDeltaTheta));
        deltaTheta = parameters.rotationFilterWeight * lastDeltaTheta ...
            + (1 - parameters.rotationFilterWeight) * targetDeltaTheta;
        lastDeltaTheta = deltaTheta;

        if norm(deltaPosition) < parameters.motionTolerance ...
                && abs(deltaTheta) < parameters.motionTolerance
            stoppedByZeroMotion = true;
            break;
        end

        currentPosition = currentPosition + deltaPosition;
        currentTheta = currentTheta + deltaTheta;
        path(end + 1, :) = [currentPosition, currentTheta]; %#ok<AGROW>
        gap = norm(goalPose(1:2) - currentPosition);
        iteration = iteration + 1;
    end

    reachedGoal = gap <= parameters.positionTolerance;
    if reachedGoal
        path(end + 1, :) = goalPose;
        stopReason = "goal_reached";
    elseif stoppedByZeroMotion
        stopReason = "zero_motion";
    else
        stopReason = "iteration_limit";
    end

    metrics = evaluate_large_item_path( ...
        path, obstacles, xLimits, yLimits, geometry);
    plan = assemble_plan(path, goalPose, reachedGoal, stopReason, ...
        iteration, metrics, parameters, geometry);
end

function force = attractive_force(position, goal, parameters)
    difference = position - goal;
    distance = norm(difference);
    switchDistance = parameters.attractionSwitchDistance;
    gain = parameters.attractiveGain;
    if distance > switchDistance
        force = -switchDistance * gain * difference / distance;
    else
        force = -gain * difference;
    end
end

function force = repulsive_force( ...
        position, obstacles, influenceDistance, gain, maximumMagnitude)
    force = zeros(1, 2);
    for obstacleIndex = 1:size(obstacles, 1)
        difference = position - obstacles(obstacleIndex, :);
        distance = norm(difference);
        if distance < influenceDistance && distance > 1e-9
            direction = difference / distance;
            magnitude = gain ...
                * (1 / distance - 1 / influenceDistance) ...
                * (1 / distance^2);
            force = force + min(magnitude, maximumMagnitude) * direction;
        end
    end
end

function plan = assemble_plan(path, goalPose, reachedGoal, stopReason, ...
        iterations, metrics, parameters, geometry)
    plan.method = "apf";
    plan.path = path;
    plan.reachedGoal = reachedGoal;
    plan.stopReason = stopReason;
    plan.endpointPositionError = norm(path(end, 1:2) - goalPose(1:2));
    plan.endpointThetaErrorDeg = abs(rad2deg(atan2( ...
        sin(path(end, 3) - goalPose(3)), ...
        cos(path(end, 3) - goalPose(3)))));
    plan.insideBoard = metrics.insideBoard;
    plan.clearanceSafe = metrics.clearanceSafe;
    plan.minimumCenterDistance = metrics.minimumCenterDistance;
    plan.minimumCornerDistance = metrics.minimumCornerDistance;
    plan.pathLength = metrics.pathLength;
    plan.combinedMotionCost = metrics.combinedMotionCost;
    plan.totalRotationDeg = metrics.totalRotationDeg;
    plan.maximumStep = metrics.maximumStep;
    plan.maximumThetaStepDeg = metrics.maximumThetaStepDeg;
    plan.maximumTurnDeg = metrics.maximumTurnDeg;
    plan.iterations = iterations;
    plan.parameters.algorithm = parameters;
    plan.parameters.geometry = geometry;
end
