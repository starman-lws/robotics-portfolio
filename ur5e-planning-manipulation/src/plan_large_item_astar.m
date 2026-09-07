function plan = plan_large_item_astar(scene)
%PLAN_LARGE_ITEM_ASTAR 使用SE(2) A*搜索并安全平滑大物体路径。
%   输出包含原始搜索节点rawPath和平滑后的N x 3 path。

    [startPose, goalPose, obstacles, xLimits, yLimits] = ...
        large_item_scene_inputs(scene);
    config = large_item_planning_config();

    [rawPath, expandedNodes] = search_large_item_astar( ...
        startPose, goalPose, obstacles, xLimits, yLimits, config);
    if isempty(rawPath)
        path = zeros(0, 3);
        reachedGoal = false;
        stopReason = "no_path";
    else
        path = smooth_large_item_astar_path( ...
            rawPath, obstacles, xLimits, yLimits, config);
        reachedGoal = true;
        stopReason = "goal_reached";
    end

    metrics = evaluate_large_item_path( ...
        path, obstacles, xLimits, yLimits, config.geometry);
    plan = assemble_plan(path, rawPath, goalPose, reachedGoal, ...
        stopReason, expandedNodes, metrics, config);
end

function plan = assemble_plan(path, rawPath, goalPose, reachedGoal, ...
        stopReason, expandedNodes, metrics, config)
    plan.method = "astar";
    plan.path = path;
    plan.reachedGoal = reachedGoal;
    plan.stopReason = stopReason;
    if isempty(path)
        plan.endpointPositionError = Inf;
        plan.endpointThetaErrorDeg = Inf;
    else
        plan.endpointPositionError = norm( ...
            path(end, 1:2) - goalPose(1:2));
        plan.endpointThetaErrorDeg = abs(rad2deg(atan2( ...
            sin(path(end, 3) - goalPose(3)), ...
            cos(path(end, 3) - goalPose(3)))));
    end
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
    plan.rawPath = rawPath;
    plan.expandedNodes = expandedNodes;
    plan.parameters.algorithm = config.astar;
    plan.parameters.smoothing = config.smoothing;
    plan.parameters.geometry = config.geometry;
end
