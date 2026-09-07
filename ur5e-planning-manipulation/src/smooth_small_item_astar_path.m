function smoothedPath = smooth_small_item_astar_path( ...
        rawPath, obstacles, xLimits, yLimits, config)
%SMOOTH_SMALL_ITEM_ASTAR_PATH 安全简化并圆滑小物体A*路径。
%   依次执行贪心shortcut、二次Bezier圆角和5 mm重新采样。所有边
%   使用相同的65 mm边界与115 mm障碍物clearance检查。

    if nargin < 5
        config = small_item_astar_config();
    end
    geometry = config.geometry;
    smoothing = config.smoothing;
    itemRadius = geometry.itemRadius;
    minimumCenterDistance = geometry.minimumCenterDistance;
    collisionResolution = geometry.collisionResolution;
    numericalTolerance = geometry.numericalTolerance;
    outputResolution = smoothing.outputResolution;
    roundingFraction = smoothing.roundingFraction;
    maximumRoundingTrim = smoothing.maximumRoundingTrim;
    maximumRoundingAttempts = smoothing.maximumRoundingAttempts;

    if isempty(rawPath)
        smoothedPath = zeros(0, 2);
        return;
    end

    rawPath = validate_path(rawPath);
    obstacles = normalise_obstacles(obstacles);
    xLimits = validate_limits(xLimits, 'xLimits');
    yLimits = validate_limits(yLimits, 'yLimits');
    safeXLimits = xLimits + [itemRadius, -itemRadius];
    safeYLimits = yLimits + [itemRadius, -itemRadius];
    if safeXLimits(1) > safeXLimits(2) ...
            || safeYLimits(1) > safeYLimits(2)
        error('smooth_small_item_astar_path:WorkspaceTooSmall', ...
              '工作区无法容纳半径65 mm的小物体。');
    end

    continuousPath = remove_consecutive_duplicates(rawPath);
    if ~path_is_valid(continuousPath)
        error('smooth_small_item_astar_path:UnsafeRawPath', ...
              'rawPath违反65 mm边界或115 mm障碍物clearance。');
    end
    if size(continuousPath, 1) == 1
        smoothedPath = continuousPath;
        return;
    end

    shortcutPath = create_shortcut_path(continuousPath);
    roundedPath = create_rounded_path(shortcutPath);
    candidatePath = resample_path(roundedPath, outputResolution);
    candidatePath(1, :) = continuousPath(1, :);
    candidatePath(end, :) = continuousPath(end, :);
    if path_is_valid(candidatePath)
        smoothedPath = candidatePath;
        return;
    end

    fallbackPath = resample_path(shortcutPath, outputResolution);
    fallbackPath(1, :) = continuousPath(1, :);
    fallbackPath(end, :) = continuousPath(end, :);
    if ~path_is_valid(fallbackPath)
        error('smooth_small_item_astar_path:UnexpectedUnsafeResult', ...
              '已经验证的shortcut路径在重采样后变得不安全。');
    end
    smoothedPath = fallbackPath;

    function shortcut = create_shortcut_path(path)
        shortcut = path(1, :);
        currentIndex = 1;
        finalIndex = size(path, 1);
        while currentIndex < finalIndex
            nextIndex = finalIndex;
            while nextIndex > currentIndex + 1 ...
                    && ~transition_is_valid( ...
                        path(currentIndex, :), path(nextIndex, :))
                nextIndex = nextIndex - 1;
            end
            shortcut(end + 1, :) = path(nextIndex, :); %#ok<AGROW>
            currentIndex = nextIndex;
        end
    end

    function rounded = create_rounded_path(path)
        if size(path, 1) <= 2
            rounded = path;
            return;
        end

        rounded = path(1, :);
        for pathIndex = 2:size(path, 1) - 1
            previousPoint = path(pathIndex - 1, :);
            cornerPoint = path(pathIndex, :);
            nextPoint = path(pathIndex + 1, :);
            previousLength = norm(cornerPoint - previousPoint);
            nextLength = norm(nextPoint - cornerPoint);
            if previousLength <= numericalTolerance ...
                    || nextLength <= numericalTolerance
                rounded = append_unique(rounded, cornerPoint);
                continue;
            end

            initialTrim = min(maximumRoundingTrim, ...
                roundingFraction * min(previousLength, nextLength));
            accepted = false;
            for attemptIndex = 0:maximumRoundingAttempts - 1
                trim = initialTrim / (2^attemptIndex);
                entryPoint = cornerPoint ...
                    + trim * (previousPoint - cornerPoint) ...
                    / previousLength;
                exitPoint = cornerPoint ...
                    + trim * (nextPoint - cornerPoint) / nextLength;
                curveSamples = sample_quadratic_bezier( ...
                    [entryPoint; cornerPoint; exitPoint]);
                if path_is_valid(curveSamples)
                    rounded = append_unique(rounded, curveSamples);
                    accepted = true;
                    break;
                end
            end
            if ~accepted
                rounded = append_unique(rounded, cornerPoint);
            end
        end
        rounded = append_unique(rounded, path(end, :));
    end

    function samples = sample_quadratic_bezier(controlPoints)
        firstEdgeLength = norm( ...
            controlPoints(2, :) - controlPoints(1, :));
        secondEdgeLength = norm( ...
            controlPoints(3, :) - controlPoints(2, :));
        sampleCount = max(2, ceil(2 * max( ...
            firstEdgeLength, secondEdgeLength) / collisionResolution));
        parameter = linspace(0, 1, sampleCount + 1).';
        samples = (1 - parameter).^2 .* controlPoints(1, :) ...
            + 2 * (1 - parameter) .* parameter .* controlPoints(2, :) ...
            + parameter.^2 .* controlPoints(3, :);
    end

    function output = resample_path(path, maximumStep)
        output = path(1, :);
        for segmentIndex = 1:size(path, 1) - 1
            fromPoint = path(segmentIndex, :);
            toPoint = path(segmentIndex + 1, :);
            distance = norm(toPoint - fromPoint);
            sampleCount = max(1, ceil(distance / maximumStep));
            for sampleIndex = 1:sampleCount
                fraction = sampleIndex / sampleCount;
                point = fromPoint + fraction * (toPoint - fromPoint);
                output = append_unique(output, point);
            end
        end
    end

    function output = append_unique(output, points)
        for pointIndex = 1:size(points, 1)
            point = points(pointIndex, :);
            if norm(output(end, :) - point) > numericalTolerance
                output(end + 1, :) = point; %#ok<AGROW>
            end
        end
    end

    function valid = path_is_valid(path)
        valid = position_is_valid(path(1, :));
        if ~valid
            return;
        end
        for segmentIndex = 1:size(path, 1) - 1
            if ~transition_is_valid( ...
                    path(segmentIndex, :), path(segmentIndex + 1, :))
                valid = false;
                return;
            end
        end
    end

    function valid = transition_is_valid(fromPoint, toPoint)
        if ~position_is_valid(fromPoint)
            valid = false;
            return;
        end
        distance = norm(toPoint - fromPoint);
        sampleCount = max(1, ceil(distance / collisionResolution));
        valid = true;
        for sampleIndex = 1:sampleCount
            fraction = sampleIndex / sampleCount;
            point = fromPoint + fraction * (toPoint - fromPoint);
            if ~position_is_valid(point)
                valid = false;
                return;
            end
        end
    end

    function valid = position_is_valid(position)
        valid = position(1) >= safeXLimits(1) - numericalTolerance ...
            && position(1) <= safeXLimits(2) + numericalTolerance ...
            && position(2) >= safeYLimits(1) - numericalTolerance ...
            && position(2) <= safeYLimits(2) + numericalTolerance;
        if ~valid || isempty(obstacles)
            return;
        end
        differences = obstacles - position;
        if any(sum(differences.^2, 2) <= minimumCenterDistance^2)
            valid = false;
        end
    end
end

function path = validate_path(value)
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) ...
            || size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('smooth_small_item_astar_path:InvalidPath', ...
              'rawPath必须是有限实数N x 2矩阵。');
    end
    path = double(value);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value(:)))
        error('smooth_small_item_astar_path:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = sort(reshape(double(value), 1, 2));
    if limits(1) >= limits(2)
        error('smooth_small_item_astar_path:InvalidLimits', ...
              '%s必须覆盖非零范围。', argumentName);
    end
end

function obstacles = normalise_obstacles(value)
    if isempty(value)
        obstacles = zeros(0, 2);
        return;
    end
    if ~isnumeric(value) || ~isreal(value) ...
            || any(~isfinite(value(:)))
        error('smooth_small_item_astar_path:InvalidObstacles', ...
              'obstacles必须包含有限实数坐标。');
    end
    if ismatrix(value) && size(value, 2) == 2
        obstacles = double(value);
    elseif size(value, 1) == 1 && size(value, 2) == 2
        obstacles = reshape(permute(double(value), [3, 2, 1]), [], 2);
    else
        error('smooth_small_item_astar_path:InvalidObstacles', ...
              'obstacles必须是N x 2或1 x 2 x N数组。');
    end
end

function path = remove_consecutive_duplicates(path)
    if size(path, 1) <= 1
        return;
    end
    changes = vecnorm(diff(path, 1, 1), 2, 2) > 1e-10;
    path = path([true; changes], :);
end
