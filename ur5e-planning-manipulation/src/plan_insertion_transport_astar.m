function plan = plan_insertion_transport_astar(scene)
%PLAN_INSERTION_TRANSPORT_ASTAR 规划小物体到倾斜槽入口的二维A*路径。
%   A*只负责桌面上方的平面运输；倾斜、插入和撤离由独立动作生成器
%   根据同一plan.path完成。

    [startPosition, goalPosition, obstacles, xLimits, yLimits] = ...
        unpack_scene(scene);
    config = small_item_astar_config();

    [rawPath, expandedNodes] = search_small_item_astar( ...
        startPosition, goalPosition, obstacles, ...
        xLimits, yLimits, config);
    if isempty(rawPath)
        path = zeros(0, 2);
        reachedGoal = false;
        stopReason = "no_path";
    else
        path = smooth_small_item_astar_path( ...
            rawPath, obstacles, xLimits, yLimits, config);
        reachedGoal = true;
        stopReason = "goal_reached";
    end

    metrics = evaluate_small_item_path( ...
        path, obstacles, xLimits, yLimits, config.geometry);
    plan = assemble_plan(path, rawPath, goalPosition, reachedGoal, ...
        stopReason, expandedNodes, metrics, config);
end

function [startPosition, goalPosition, obstacles, xLimits, yLimits] = ...
        unpack_scene(scene)
    requiredFields = {'smallItem', 'slotEntrance', ...
        'obstacles', 'boardReference'};
    if ~isstruct(scene) || ~isscalar(scene) ...
            || ~all(isfield(scene, requiredFields))
        error('plan_insertion_transport_astar:InvalidScene', ...
              'scene缺少倾斜插入A*规划所需字段。');
    end

    startPosition = unpack_position( ...
        scene.smallItem, 2, 'scene.smallItem');
    slotPosition = unpack_position( ...
        scene.slotEntrance, 3, 'scene.slotEntrance');
    goalPosition = slotPosition(1:2);

    if ~isstruct(scene.obstacles) ...
            || ~isfield(scene.obstacles, 'position')
        error('plan_insertion_transport_astar:InvalidScene', ...
              'scene.obstacles缺少position。');
    end
    obstacles = scene.obstacles.position;
    if isempty(obstacles)
        obstacles = zeros(0, 2);
    elseif ~isnumeric(obstacles) || ~isreal(obstacles) ...
            || ~ismatrix(obstacles) || size(obstacles, 2) ~= 2 ...
            || any(~isfinite(obstacles(:)))
        error('plan_insertion_transport_astar:InvalidScene', ...
              'scene.obstacles.position必须是N x 2有限实数矩阵。');
    else
        obstacles = double(obstacles);
    end

    try
        xLimits = double(scene.boardReference.XWorldLimits);
        yLimits = double(scene.boardReference.YWorldLimits);
    catch
        error('plan_insertion_transport_astar:InvalidBoardReference', ...
              'scene.boardReference必须提供XWorldLimits和YWorldLimits。');
    end
end

function position = unpack_position(pose, dimension, argumentName)
    if ~isstruct(pose) || ~isscalar(pose) ...
            || ~isfield(pose, 'position')
        error('plan_insertion_transport_astar:InvalidScene', ...
              '%s缺少position。', argumentName);
    end
    position = pose.position;
    if ~isnumeric(position) || ~isreal(position) ...
            || ~isequal(size(position), [1, dimension]) ...
            || any(~isfinite(position(:)))
        error('plan_insertion_transport_astar:InvalidScene', ...
              '%s.position必须是1x%d有限实数向量。', ...
              argumentName, dimension);
    end
    position = double(position);
end

function plan = assemble_plan(path, rawPath, goalPosition, ...
        reachedGoal, stopReason, expandedNodes, metrics, config)
    plan.method = "astar";
    plan.path = path;
    plan.rawPath = rawPath;
    plan.reachedGoal = reachedGoal;
    plan.stopReason = stopReason;
    if isempty(path)
        plan.endpointError = Inf;
    else
        plan.endpointError = norm(path(end, :) - goalPosition);
    end
    plan.insideBoard = metrics.insideBoard;
    plan.clearanceSafe = metrics.clearanceSafe;
    plan.minimumObstacleDistance = metrics.minimumObstacleDistance;
    plan.pathLength = metrics.pathLength;
    plan.meanStep = metrics.meanStep;
    plan.maximumStep = metrics.maximumStep;
    plan.maximumTurnDeg = metrics.maximumTurnDeg;
    plan.expandedNodes = expandedNodes;
    plan.parameters.algorithm = config.algorithm;
    plan.parameters.smoothing = config.smoothing;
    plan.parameters.geometry = config.geometry;
end
