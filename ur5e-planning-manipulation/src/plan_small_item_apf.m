function plan = plan_small_item_apf(scene)
%PLAN_SMALL_ITEM_APF 为平面小物体生成APF势场和中心路径。
%   输入scene由LOCALIZE_PICK_PLACE_SCENE生成。输出包含势场采样、路径、
%   停止原因和中心点诊断，不表示物体实体几何的碰撞安全保证。

    [startPosition, goalPosition, obstacles, xLimits, yLimits] = ...
        unpack_scene(scene);
    parameters = small_item_apf_config();
    field = sample_apf_vector_field( ...
        goalPosition, obstacles, xLimits, yLimits, parameters);

    currentPosition = startPosition;
    path = currentPosition;
    gap = norm(goalPosition - currentPosition);
    iteration = 0;
    lastDelta = [0, 0];
    stoppedByZeroForce = false;

    while gap > parameters.step ...
            && iteration < parameters.maximumIterations
        force = apf_total_force( ...
            currentPosition, goalPosition, obstacles, parameters);
        forceMagnitude = norm(force);
        if forceMagnitude < parameters.forceTolerance
            stoppedByZeroForce = true;
            break;
        end

        targetDelta = parameters.step * force / forceMagnitude;
        delta = parameters.smoothingFactor * lastDelta ...
              + (1 - parameters.smoothingFactor) * targetDelta;
        currentPosition = currentPosition + delta;
        path(end + 1, :) = currentPosition; %#ok<AGROW>
        lastDelta = delta;
        gap = norm(goalPosition - currentPosition);
        iteration = iteration + 1;
    end

    reachedGoal = gap <= parameters.step;
    if reachedGoal
        if gap > parameters.forceTolerance
            path(end + 1, :) = goalPosition;
        else
            path(end, :) = goalPosition;
        end
        stopReason = "goal_reached";
    elseif stoppedByZeroForce
        stopReason = "zero_force";
    else
        stopReason = "iteration_limit";
    end

    [meanStep, maximumStep, maximumTurnDeg] = path_metrics(path);
    insideBoard = all(path(:, 1) >= xLimits(1)) ...
        && all(path(:, 1) <= xLimits(2)) ...
        && all(path(:, 2) >= yLimits(1)) ...
        && all(path(:, 2) <= yLimits(2));
    minimumObstacleDistance = minimum_center_distance(path, obstacles);

    plan.path = path;
    plan.field = field;
    plan.reachedGoal = reachedGoal;
    plan.stopReason = stopReason;
    plan.iterations = iteration;
    plan.endpointError = norm(path(end, :) - goalPosition);
    plan.insideBoard = insideBoard;
    plan.minimumObstacleDistance = minimumObstacleDistance;
    plan.meanStep = meanStep;
    plan.maximumStep = maximumStep;
    plan.maximumTurnDeg = maximumTurnDeg;
    plan.parameters = parameters;
end

function [startPosition, goalPosition, obstacles, xLimits, yLimits] = ...
        unpack_scene(scene)
    requiredFields = {'smallItem', 'smallGoal', ...
                      'obstacles', 'boardReference'};
    if ~isstruct(scene) || ~isscalar(scene) || ...
            ~all(isfield(scene, requiredFields))
        error('plan_small_item_apf:InvalidScene', ...
              'scene缺少小物体APF规划所需字段。');
    end

    if ~isstruct(scene.smallItem) ...
            || ~isfield(scene.smallItem, 'position')
        error('plan_small_item_apf:InvalidScene', ...
              'scene.smallItem缺少position。');
    end
    if ~isstruct(scene.smallGoal) ...
            || ~isfield(scene.smallGoal, 'position')
        error('plan_small_item_apf:InvalidScene', ...
              'scene.smallGoal缺少position。');
    end
    if ~isstruct(scene.obstacles) ...
            || ~isfield(scene.obstacles, 'position')
        error('plan_small_item_apf:InvalidScene', ...
              'scene.obstacles缺少position。');
    end

    startPosition = normalise_single_position( ...
        scene.smallItem.position, 'scene.smallItem.position');
    goalPosition = normalise_single_position( ...
        scene.smallGoal.position, 'scene.smallGoal.position');
    obstacles = normalise_obstacles(scene.obstacles.position);

    try
        xLimits = double(scene.boardReference.XWorldLimits);
        yLimits = double(scene.boardReference.YWorldLimits);
    catch
        error('plan_small_item_apf:InvalidBoardReference', ...
              'scene.boardReference必须提供XWorldLimits和YWorldLimits。');
    end

    if ~valid_limits(xLimits) || ~valid_limits(yLimits)
        error('plan_small_item_apf:InvalidBoardReference', ...
              '工作区范围必须是递增的有限实数向量。');
    end
end

function position = normalise_single_position(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ...
            ~isequal(size(value), [1, 2]) || any(~isfinite(value(:)))
        error('plan_small_item_apf:InvalidPosition', ...
              '%s必须是1x2有限实数向量。', argumentName);
    end
    position = double(value);
end

function obstacles = normalise_obstacles(value)
    if isempty(value)
        obstacles = zeros(0, 2);
        return;
    end
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) || ...
            size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('plan_small_item_apf:InvalidObstacles', ...
              'scene.obstacles.position必须是Nx2有限实数矩阵或空数组。');
    end
    obstacles = double(value);
end

function valid = valid_limits(limits)
    valid = isnumeric(limits) && isreal(limits) ...
        && isvector(limits) && numel(limits) == 2 ...
        && all(isfinite(limits)) && limits(1) < limits(2);
end

function [meanStep, maximumStep, maximumTurnDeg] = path_metrics(path)
    segments = diff(path, 1, 1);
    lengths = vecnorm(segments, 2, 2);
    if isempty(lengths)
        meanStep = 0;
        maximumStep = 0;
        maximumTurnDeg = 0;
        return;
    end

    meanStep = mean(lengths);
    maximumStep = max(lengths);
    nonzeroSegments = segments(lengths > 1e-12, :);
    if size(nonzeroSegments, 1) < 2
        maximumTurnDeg = 0;
        return;
    end

    unitSegments = nonzeroSegments ...
        ./ vecnorm(nonzeroSegments, 2, 2);
    cosine = sum(unitSegments(1:end - 1, :) ...
        .* unitSegments(2:end, :), 2);
    cosine = max(-1, min(1, cosine));
    maximumTurnDeg = max(rad2deg(acos(cosine)));
end

function minimumDistance = minimum_center_distance(path, obstacles)
    if isempty(obstacles)
        minimumDistance = Inf;
        return;
    end

    minimumDistance = Inf;
    for obstacleIndex = 1:size(obstacles, 1)
        distances = vecnorm( ...
            path - obstacles(obstacleIndex, :), 2, 2);
        minimumDistance = min(minimumDistance, min(distances));
    end
end
