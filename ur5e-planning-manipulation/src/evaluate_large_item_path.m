function metrics = evaluate_large_item_path( ...
        path, obstacles, xLimits, yLimits, geometry)
%EVALUATE_LARGE_ITEM_PATH 评估大物体SE(2)路径的五点几何和运动指标。
%   碰撞诊断使用物体中心与四角点，不建模矩形边缘。

    if nargin < 5
        config = large_item_planning_config();
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

    metrics.insideBoard = true;
    metrics.minimumCenterDistance = Inf;
    metrics.minimumCornerDistance = Inf;

    numberOfSegments = max(1, size(path, 1) - 1);
    for segmentIndex = 1:numberOfSegments
        if size(path, 1) == 1
            fromPose = path(1, :);
            toPose = path(1, :);
        else
            fromPose = path(segmentIndex, :);
            toPose = path(segmentIndex + 1, :);
        end

        translationDistance = norm(toPose(1:2) - fromPose(1:2));
        thetaChange = toPose(3) - fromPose(3);
        sampleCount = max(1, ceil(max( ...
            translationDistance / geometry.collisionXYResolution, ...
            abs(thetaChange) / geometry.collisionThetaResolution)));

        for sampleIndex = 0:sampleCount
            fraction = sampleIndex / sampleCount;
            pose = [fromPose(1:2) ...
                + fraction * (toPose(1:2) - fromPose(1:2)), ...
                fromPose(3) + fraction * thetaChange];
            corners = large_item_corners(pose, geometry.localCorners);
            metrics.insideBoard = metrics.insideBoard ...
                && all(corners(:, 1) >= xLimits(1)) ...
                && all(corners(:, 1) <= xLimits(2)) ...
                && all(corners(:, 2) >= yLimits(1)) ...
                && all(corners(:, 2) <= yLimits(2));

            if ~isempty(obstacles)
                centerDistances = vecnorm(obstacles - pose(1:2), 2, 2);
                metrics.minimumCenterDistance = min( ...
                    metrics.minimumCenterDistance, min(centerDistances));
                for cornerIndex = 1:size(corners, 1)
                    cornerDistances = vecnorm( ...
                        obstacles - corners(cornerIndex, :), 2, 2);
                    metrics.minimumCornerDistance = min( ...
                        metrics.minimumCornerDistance, ...
                        min(cornerDistances));
                end
            end
        end
    end

    metrics.clearanceSafe = metrics.insideBoard ...
        && metrics.minimumCenterDistance ...
            > geometry.minimumCenterDistance ...
        && metrics.minimumCornerDistance ...
            > geometry.minimumCornerDistance;
    metrics = add_motion_metrics(metrics, path, geometry.cornerRadius);
end

function metrics = empty_metrics()
    metrics.insideBoard = false;
    metrics.clearanceSafe = false;
    metrics.minimumCenterDistance = NaN;
    metrics.minimumCornerDistance = NaN;
    metrics.pathLength = NaN;
    metrics.combinedMotionCost = NaN;
    metrics.totalRotationDeg = NaN;
    metrics.maximumStep = NaN;
    metrics.maximumThetaStepDeg = NaN;
    metrics.maximumTurnDeg = NaN;
end

function metrics = add_motion_metrics(metrics, path, cornerRadius)
    segments = diff(path, 1, 1);
    translationSteps = vecnorm(segments(:, 1:2), 2, 2);
    thetaSteps = abs(segments(:, 3));

    metrics.pathLength = sum(translationSteps);
    metrics.combinedMotionCost = sum(hypot( ...
        translationSteps, cornerRadius * thetaSteps));
    metrics.totalRotationDeg = sum(rad2deg(thetaSteps));
    if isempty(translationSteps)
        metrics.maximumStep = 0;
        metrics.maximumThetaStepDeg = 0;
        metrics.maximumTurnDeg = 0;
        return;
    end

    metrics.maximumStep = max(translationSteps);
    metrics.maximumThetaStepDeg = max(rad2deg(thetaSteps));
    movingSegments = segments(translationSteps > 1e-9, 1:2);
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
            || size(value, 2) ~= 3 || any(~isfinite(value(:)))
        error('evaluate_large_item_path:InvalidPath', ...
              'path必须是有限实数N x 3矩阵。');
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
        error('evaluate_large_item_path:InvalidObstacles', ...
              'obstacles必须是有限实数N x 2矩阵或空数组。');
    end
    obstacles = double(value);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) ...
            || numel(value) ~= 2 || any(~isfinite(value(:)))
        error('evaluate_large_item_path:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = double(value(:).');
    if limits(1) >= limits(2)
        error('evaluate_large_item_path:InvalidLimits', ...
              '%s的下限必须小于上限。', argumentName);
    end
end

function validate_geometry(geometry)
    requiredFields = {'localCorners', 'cornerRadius', ...
        'minimumCenterDistance', 'minimumCornerDistance', ...
        'collisionXYResolution', 'collisionThetaResolution'};
    if ~isstruct(geometry) || ~isscalar(geometry) ...
            || ~all(isfield(geometry, requiredFields))
        error('evaluate_large_item_path:InvalidGeometry', ...
              'geometry缺少路径诊断所需字段。');
    end
    values = [geometry.cornerRadius, geometry.minimumCenterDistance, ...
        geometry.minimumCornerDistance, ...
        geometry.collisionXYResolution, ...
        geometry.collisionThetaResolution];
    if ~isnumeric(values) || ~isreal(values) ...
            || any(~isfinite(values)) || any(values <= 0)
        error('evaluate_large_item_path:InvalidGeometry', ...
              'geometry中的距离与分辨率必须是正的有限实数。');
    end
end
