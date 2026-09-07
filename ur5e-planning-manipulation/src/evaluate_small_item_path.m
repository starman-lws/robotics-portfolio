function metrics = evaluate_small_item_path( ...
        path, obstacles, xLimits, yLimits, geometry)
%EVALUATE_SMALL_ITEM_PATH 连续评估小物体中心路径和运动指标。
%   小物体与障碍物使用半径65 mm和50 mm的圆形近似，路径边按不超过
%   2.5 mm的间隔插值。

    if nargin < 5
        config = small_item_astar_config();
        geometry = config.geometry;
    end
    path = validate_path(path);
    obstacles = normalise_obstacles(obstacles);
    xLimits = validate_limits(xLimits, 'xLimits');
    yLimits = validate_limits(yLimits, 'yLimits');
    validate_geometry(geometry);

    metrics = empty_metrics();
    if isempty(path)
        return;
    end

    safeXLimits = xLimits + [geometry.itemRadius, -geometry.itemRadius];
    safeYLimits = yLimits + [geometry.itemRadius, -geometry.itemRadius];
    metrics.insideBoard = true;
    metrics.minimumObstacleDistance = Inf;

    segmentCount = max(1, size(path, 1) - 1);
    for segmentIndex = 1:segmentCount
        if size(path, 1) == 1
            fromPoint = path(1, :);
            toPoint = path(1, :);
        else
            fromPoint = path(segmentIndex, :);
            toPoint = path(segmentIndex + 1, :);
        end
        distance = norm(toPoint - fromPoint);
        sampleCount = max(1, ...
            ceil(distance / geometry.collisionResolution));
        for sampleIndex = 0:sampleCount
            fraction = sampleIndex / sampleCount;
            point = fromPoint + fraction * (toPoint - fromPoint);
            metrics.insideBoard = metrics.insideBoard ...
                && point(1) >= safeXLimits(1) ...
                && point(1) <= safeXLimits(2) ...
                && point(2) >= safeYLimits(1) ...
                && point(2) <= safeYLimits(2);
            if ~isempty(obstacles)
                distances = vecnorm(obstacles - point, 2, 2);
                metrics.minimumObstacleDistance = min( ...
                    metrics.minimumObstacleDistance, min(distances));
            end
        end
    end

    metrics.clearanceSafe = metrics.insideBoard ...
        && metrics.minimumObstacleDistance ...
            > geometry.minimumCenterDistance;
    metrics = add_motion_metrics(metrics, path);
end

function metrics = empty_metrics()
    metrics.insideBoard = false;
    metrics.clearanceSafe = false;
    metrics.minimumObstacleDistance = NaN;
    metrics.pathLength = NaN;
    metrics.meanStep = NaN;
    metrics.maximumStep = NaN;
    metrics.maximumTurnDeg = NaN;
end

function metrics = add_motion_metrics(metrics, path)
    segments = diff(path, 1, 1);
    lengths = vecnorm(segments, 2, 2);
    metrics.pathLength = sum(lengths);
    if isempty(lengths)
        metrics.meanStep = 0;
        metrics.maximumStep = 0;
        metrics.maximumTurnDeg = 0;
        return;
    end

    metrics.meanStep = mean(lengths);
    metrics.maximumStep = max(lengths);
    movingSegments = segments(lengths > 1e-12, :);
    if size(movingSegments, 1) < 2
        metrics.maximumTurnDeg = 0;
        return;
    end
    headings = atan2(movingSegments(:, 2), movingSegments(:, 1));
    turns = atan2(sin(diff(headings)), cos(diff(headings)));
    metrics.maximumTurnDeg = max(abs(rad2deg(turns)));
end

function path = validate_path(value)
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) ...
            || size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('evaluate_small_item_path:InvalidPath', ...
              'path必须是有限实数N x 2矩阵。');
    end
    path = double(value);
end

function obstacles = normalise_obstacles(value)
    if isempty(value)
        obstacles = zeros(0, 2);
        return;
    end
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) ...
            || size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('evaluate_small_item_path:InvalidObstacles', ...
              'obstacles必须是有限实数N x 2矩阵或空数组。');
    end
    obstacles = double(value);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value(:)))
        error('evaluate_small_item_path:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = sort(reshape(double(value), 1, 2));
    if limits(1) >= limits(2)
        error('evaluate_small_item_path:InvalidLimits', ...
              '%s必须覆盖非零范围。', argumentName);
    end
end

function validate_geometry(geometry)
    requiredFields = {'itemRadius', 'obstacleRadius', ...
        'minimumCenterDistance', 'collisionResolution', ...
        'numericalTolerance'};
    if ~isstruct(geometry) || ~isscalar(geometry) ...
            || ~all(isfield(geometry, requiredFields))
        error('evaluate_small_item_path:InvalidGeometry', ...
              'geometry缺少小物体路径诊断所需字段。');
    end
    values = [geometry.itemRadius, geometry.obstacleRadius, ...
        geometry.minimumCenterDistance, geometry.collisionResolution];
    if any(~isfinite(values)) || any(values <= 0) ...
            || ~isfinite(geometry.numericalTolerance) ...
            || geometry.numericalTolerance < 0
        error('evaluate_small_item_path:InvalidGeometry', ...
              'geometry中的半径和分辨率必须是有效有限实数。');
    end
end
