function smoothedPath = smooth_large_item_astar_path( ...
        rawPath, obstacles, xLimits, yLimits, config)
%SMOOTH_LARGE_ITEM_ASTAR_PATH 安全简化并圆滑大物体A*位姿路径。
%   依次执行shortcut、二次Bezier局部圆角和5 mm / 3 degree重采样。

    if nargin < 5
        config = large_item_planning_config();
    end
    if isempty(rawPath)
        smoothedPath = zeros(0, 3);
        return;
    end

    rawPath = validate_path(rawPath);
    obstacles = normalise_obstacles(obstacles);
    xLimits = validate_limits(xLimits, 'xLimits');
    yLimits = validate_limits(yLimits, 'yLimits');
    geometry = config.geometry;
    smoothing = config.smoothing;
    collisionXYResolution = geometry.collisionXYResolution;
    collisionThetaResolution = geometry.collisionThetaResolution;
    numericalTolerance = geometry.numericalTolerance;
    thetaScale = geometry.cornerRadius;

    continuousPath = rawPath;
    continuousPath(:, 3) = unwrap(continuousPath(:, 3));
    continuousPath = remove_consecutive_duplicates(continuousPath);
    if ~path_is_valid(continuousPath)
        error('smooth_large_item_astar_path:UnsafeRawPath', ...
              'rawPath违反工作区或100/70 mm五点距离约束。');
    end
    if size(continuousPath, 1) == 1
        smoothedPath = continuousPath;
        return;
    end

    shortcutPath = create_shortcut_path(continuousPath);
    roundedPath = create_rounded_path(shortcutPath);
    candidatePath = resample_path(roundedPath, ...
        smoothing.outputXYResolution, ...
        smoothing.outputThetaResolution);
    candidatePath(1, :) = continuousPath(1, :);
    candidatePath(end, :) = continuousPath(end, :);
    if path_is_valid(candidatePath)
        smoothedPath = candidatePath;
        return;
    end

    fallbackPath = resample_path(shortcutPath, ...
        smoothing.outputXYResolution, ...
        smoothing.outputThetaResolution);
    fallbackPath(1, :) = continuousPath(1, :);
    fallbackPath(end, :) = continuousPath(end, :);
    if ~path_is_valid(fallbackPath)
        error('smooth_large_item_astar_path:UnexpectedUnsafeResult', ...
              '已验证的shortcut路径在重采样后变得不安全。');
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

        scaledPath = [path(:, 1:2), thetaScale * path(:, 3)];
        rounded = path(1, :);
        for pathIndex = 2:size(path, 1) - 1
            previousScaled = scaledPath(pathIndex - 1, :);
            cornerScaled = scaledPath(pathIndex, :);
            nextScaled = scaledPath(pathIndex + 1, :);
            previousLength = norm(cornerScaled - previousScaled);
            nextLength = norm(nextScaled - cornerScaled);
            if previousLength <= numericalTolerance ...
                    || nextLength <= numericalTolerance
                rounded = append_unique(rounded, path(pathIndex, :));
                continue;
            end

            initialTrim = min(smoothing.maximumRoundingTrim, ...
                smoothing.roundingFraction ...
                * min(previousLength, nextLength));
            accepted = false;
            for attemptIndex = 0:smoothing.maximumRoundingAttempts - 1
                trim = initialTrim / (2^attemptIndex);
                entryScaled = cornerScaled + trim ...
                    * (previousScaled - cornerScaled) / previousLength;
                exitScaled = cornerScaled + trim ...
                    * (nextScaled - cornerScaled) / nextLength;
                controlPoses = [scaled_to_pose(entryScaled); ...
                    path(pathIndex, :); scaled_to_pose(exitScaled)];
                curveSamples = sample_quadratic_bezier(controlPoses);
                if path_is_valid(curveSamples)
                    rounded = append_unique(rounded, curveSamples);
                    accepted = true;
                    break;
                end
            end
            if ~accepted
                rounded = append_unique(rounded, path(pathIndex, :));
            end
        end
        rounded = append_unique(rounded, path(end, :));
    end

    function samples = sample_quadratic_bezier(controlPoses)
        firstEdgeXY = norm( ...
            controlPoses(2, 1:2) - controlPoses(1, 1:2));
        secondEdgeXY = norm( ...
            controlPoses(3, 1:2) - controlPoses(2, 1:2));
        firstEdgeTheta = abs(controlPoses(2, 3) - controlPoses(1, 3));
        secondEdgeTheta = abs(controlPoses(3, 3) - controlPoses(2, 3));
        sampleCount = max([2, ...
            ceil(2 * max(firstEdgeXY, secondEdgeXY) ...
                / collisionXYResolution), ...
            ceil(2 * max(firstEdgeTheta, secondEdgeTheta) ...
                / collisionThetaResolution)]);
        parameter = linspace(0, 1, sampleCount + 1).';
        samples = (1 - parameter).^2 .* controlPoses(1, :) ...
            + 2 * (1 - parameter) .* parameter .* controlPoses(2, :) ...
            + parameter.^2 .* controlPoses(3, :);
    end

    function pose = scaled_to_pose(scaledPose)
        pose = [scaledPose(1:2), scaledPose(3) / thetaScale];
    end

    function output = resample_path(path, maximumXYStep, maximumThetaStep)
        output = path(1, :);
        for segmentIndex = 1:size(path, 1) - 1
            fromPose = path(segmentIndex, :);
            toPose = path(segmentIndex + 1, :);
            translationDistance = norm( ...
                toPose(1:2) - fromPose(1:2));
            thetaChange = toPose(3) - fromPose(3);
            sampleCount = max(1, ceil(max( ...
                translationDistance / maximumXYStep, ...
                abs(thetaChange) / maximumThetaStep)));
            for sampleIndex = 1:sampleCount
                fraction = sampleIndex / sampleCount;
                pose = [fromPose(1:2) ...
                    + fraction * (toPose(1:2) - fromPose(1:2)), ...
                    fromPose(3) + fraction * thetaChange];
                output = append_unique(output, pose);
            end
        end
    end

    function output = append_unique(output, poses)
        for poseIndex = 1:size(poses, 1)
            pose = poses(poseIndex, :);
            if norm(output(end, 1:2) - pose(1:2)) ...
                    > numericalTolerance ...
                    || abs(output(end, 3) - pose(3)) ...
                        > numericalTolerance
                output(end + 1, :) = pose; %#ok<AGROW>
            end
        end
    end

    function valid = path_is_valid(path)
        valid = configuration_is_valid(path(1, :));
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

    function valid = transition_is_valid(fromPose, toPose)
        if ~configuration_is_valid(fromPose)
            valid = false;
            return;
        end
        translationDistance = norm(toPose(1:2) - fromPose(1:2));
        thetaChange = toPose(3) - fromPose(3);
        sampleCount = max(1, ceil(max( ...
            translationDistance / collisionXYResolution, ...
            abs(thetaChange) / collisionThetaResolution)));
        valid = true;
        for sampleIndex = 1:sampleCount
            fraction = sampleIndex / sampleCount;
            pose = [fromPose(1:2) ...
                + fraction * (toPose(1:2) - fromPose(1:2)), ...
                fromPose(3) + fraction * thetaChange];
            if ~configuration_is_valid(pose)
                valid = false;
                return;
            end
        end
    end

    function valid = configuration_is_valid(pose)
        corners = large_item_corners(pose, geometry.localCorners);
        valid = all(corners(:, 1) ...
                >= xLimits(1) - numericalTolerance) ...
            && all(corners(:, 1) ...
                <= xLimits(2) + numericalTolerance) ...
            && all(corners(:, 2) ...
                >= yLimits(1) - numericalTolerance) ...
            && all(corners(:, 2) ...
                <= yLimits(2) + numericalTolerance);
        if ~valid || isempty(obstacles)
            return;
        end
        centerDifferences = obstacles - pose(1:2);
        if any(sum(centerDifferences.^2, 2) ...
                <= geometry.minimumCenterDistance^2)
            valid = false;
            return;
        end
        for cornerIndex = 1:size(corners, 1)
            differences = obstacles - corners(cornerIndex, :);
            if any(sum(differences.^2, 2) ...
                    <= geometry.minimumCornerDistance^2)
                valid = false;
                return;
            end
        end
    end
end

function path = validate_path(value)
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) ...
            || size(value, 2) ~= 3 || any(~isfinite(value(:)))
        error('smooth_large_item_astar_path:InvalidPath', ...
              'rawPath必须是有限实数N x 3矩阵。');
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
        error('smooth_large_item_astar_path:InvalidObstacles', ...
              'obstacles必须是有限实数N x 2矩阵或空数组。');
    end
    obstacles = double(value);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) ...
            || numel(value) ~= 2 || any(~isfinite(value(:)))
        error('smooth_large_item_astar_path:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = double(value(:).');
    if limits(1) >= limits(2)
        error('smooth_large_item_astar_path:InvalidLimits', ...
              '%s的下限必须小于上限。', argumentName);
    end
end

function path = remove_consecutive_duplicates(path)
    if size(path, 1) <= 1
        return;
    end
    changes = vecnorm(diff(path(:, 1:2), 1, 1), 2, 2) > 1e-10 ...
        | abs(diff(path(:, 3), 1, 1)) > 1e-10;
    path = path([true; changes], :);
end
