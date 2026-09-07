function [path, expandedNodes] = search_large_item_astar( ...
        startPose, goalPose, obstacles, xLimits, yLimits, config)
%SEARCH_LARGE_ITEM_ASTAR 在5 mm / 3 degree SE(2)网格中搜索原始路径。
%   返回路径保持连续theta，并以精确输入起点和目标点为端点。

    if nargin < 6
        config = large_item_planning_config();
    end
    startPose = validate_pose(startPose, 'startPose');
    goalPose = validate_pose(goalPose, 'goalPose');
    obstacles = normalise_obstacles(obstacles);
    xLimits = validate_limits(xLimits, 'xLimits');
    yLimits = validate_limits(yLimits, 'yLimits');

    astar = config.astar;
    geometry = config.geometry;
    xyResolution = astar.xyResolution;
    thetaResolution = astar.thetaResolution;
    transitionXYResolution = astar.transitionXYResolution;
    transitionThetaResolution = astar.transitionThetaResolution;
    numericalTolerance = geometry.numericalTolerance;
    cornerRadius = geometry.cornerRadius;

    if ~configuration_is_valid(startPose)
        error('search_large_item_astar:InvalidStart', ...
              '起始位姿违反工作区或100/70 mm五点距离约束。');
    end
    if ~configuration_is_valid(goalPose)
        error('search_large_item_astar:InvalidGoal', ...
              '目标位姿违反工作区或100/70 mm五点距离约束。');
    end

    xOffsetMinimum = ceil((xLimits(1) - startPose(1)) ...
        / xyResolution - numericalTolerance);
    xOffsetMaximum = floor((xLimits(2) - startPose(1)) ...
        / xyResolution + numericalTolerance);
    yOffsetMinimum = ceil((yLimits(1) - startPose(2)) ...
        / xyResolution - numericalTolerance);
    yOffsetMaximum = floor((yLimits(2) - startPose(2)) ...
        / xyResolution + numericalTolerance);
    xOffsets = xOffsetMinimum:xOffsetMaximum;
    yOffsets = yOffsetMinimum:yOffsetMaximum;
    xGrid = startPose(1) + xyResolution * xOffsets;
    yGrid = startPose(2) + xyResolution * yOffsets;
    startXIndex = find(xOffsets == 0, 1);
    startYIndex = find(yOffsets == 0, 1);
    if isempty(startXIndex) || isempty(startYIndex)
        error('search_large_item_astar:StartOutsideGrid', ...
              '起始位置位于给定工作区之外。');
    end

    thetaBinCount = round(2 * pi / thetaResolution);
    thetaGrid = wrap_angle(startPose(3) ...
        + (0:thetaBinCount - 1) * thetaResolution);
    startThetaIndex = 1;
    gridSize = [numel(xGrid), numel(yGrid), thetaBinCount];
    stateCount = prod(double(gridSize));
    if stateCount > double(intmax('uint32'))
        error('search_large_item_astar:GridTooLarge', ...
              '搜索网格超过uint32索引容量。');
    end

    gScore = inf(gridSize);
    parent = zeros(gridSize, 'uint32');
    closed = false(gridSize);
    startNode = uint32(sub2ind(gridSize, ...
        startXIndex, startYIndex, startThetaIndex));
    gScore(double(startNode)) = 0;

    [actionX, actionY, actionTheta] = ndgrid(-1:1, -1:1, -1:1);
    actions = [actionX(:), actionY(:), actionTheta(:)];
    actions(all(actions == 0, 2), :) = [];

    heapCapacity = 4096;
    heapNodes = zeros(heapCapacity, 1, 'uint32');
    heapPriorities = inf(heapCapacity, 1);
    heapGScores = inf(heapCapacity, 1);
    heapSize = 0;
    heap_push(startNode, heuristic(startPose), 0);

    bestGoalCost = inf;
    bestGoalParent = uint32(0);
    expandedNodes = 0;

    while heapSize > 0
        [currentNode, currentPriority, queuedGScore] = heap_pop();
        if currentPriority >= bestGoalCost - numericalTolerance
            break;
        end
        currentLinearIndex = double(currentNode);
        if closed(currentLinearIndex)
            continue;
        end
        if queuedGScore > gScore(currentLinearIndex) + numericalTolerance
            continue;
        end

        closed(currentLinearIndex) = true;
        expandedNodes = expandedNodes + 1;
        [currentXIndex, currentYIndex, currentThetaIndex] = ...
            ind2sub(gridSize, currentLinearIndex);
        currentPose = [xGrid(currentXIndex), yGrid(currentYIndex), ...
            thetaGrid(currentThetaIndex)];

        goalThetaFromCurrent = currentPose(3) ...
            + angle_difference(goalPose(3), currentPose(3));
        goalConnectorPose = [goalPose(1:2), goalThetaFromCurrent];
        positionToGoal = norm(goalPose(1:2) - currentPose(1:2));
        angleToGoal = abs(goalThetaFromCurrent - currentPose(3));
        if positionToGoal <= xyResolution + numericalTolerance ...
                && angleToGoal <= thetaResolution + numericalTolerance ...
                && transition_is_valid(currentPose, goalConnectorPose)
            connectorCost = motion_cost(currentPose, goalConnectorPose);
            candidateGoalCost = gScore(currentLinearIndex) + connectorCost;
            if candidateGoalCost < bestGoalCost
                bestGoalCost = candidateGoalCost;
                bestGoalParent = currentNode;
            end
        end

        for actionIndex = 1:size(actions, 1)
            action = actions(actionIndex, :);
            nextXIndex = currentXIndex + action(1);
            nextYIndex = currentYIndex + action(2);
            if nextXIndex < 1 || nextXIndex > gridSize(1) ...
                    || nextYIndex < 1 || nextYIndex > gridSize(2)
                continue;
            end

            nextThetaIndex = mod(currentThetaIndex - 1 + action(3), ...
                thetaBinCount) + 1;
            nextNode = uint32(sub2ind(gridSize, ...
                nextXIndex, nextYIndex, nextThetaIndex));
            nextLinearIndex = double(nextNode);
            if closed(nextLinearIndex)
                continue;
            end

            nextPoseContinuous = [xGrid(nextXIndex), yGrid(nextYIndex), ...
                currentPose(3) + action(3) * thetaResolution];
            if ~transition_is_valid(currentPose, nextPoseContinuous)
                continue;
            end

            tentativeGScore = gScore(currentLinearIndex) ...
                + motion_cost(currentPose, nextPoseContinuous);
            if tentativeGScore + numericalTolerance ...
                    < gScore(nextLinearIndex)
                gScore(nextLinearIndex) = tentativeGScore;
                parent(nextLinearIndex) = currentNode;
                storedNextPose = [xGrid(nextXIndex), yGrid(nextYIndex), ...
                    thetaGrid(nextThetaIndex)];
                estimatedTotalCost = tentativeGScore ...
                    + heuristic(storedNextPose);
                heap_push(nextNode, estimatedTotalCost, tentativeGScore);
            end
        end

        if expandedNodes >= stateCount
            break;
        end
    end

    if bestGoalParent == 0
        path = zeros(0, 3);
        warning('search_large_item_astar:NoPath', ...
                '在5 mm / 3 degree网格中未找到有效SE(2)路径。');
        return;
    end

    nodePath = bestGoalParent;
    node = bestGoalParent;
    while node ~= startNode
        node = parent(double(node));
        if node == 0
            error('search_large_item_astar:BrokenParentChain', ...
                  'A* parent路径重建失败。');
        end
        nodePath(end + 1, 1) = node; %#ok<AGROW>
    end
    nodePath = flipud(nodePath);

    path = zeros(numel(nodePath), 3);
    for pathIndex = 1:numel(nodePath)
        [xIndex, yIndex, thetaIndex] = ind2sub( ...
            gridSize, double(nodePath(pathIndex)));
        path(pathIndex, :) = [xGrid(xIndex), yGrid(yIndex), ...
            thetaGrid(thetaIndex)];
    end

    path(:, 3) = unwrap(path(:, 3));
    path(:, 3) = path(:, 3) + (startPose(3) - path(1, 3));
    path(1, :) = startPose;
    continuousGoalTheta = path(end, 3) ...
        + angle_difference(goalPose(3), path(end, 3));
    exactGoalPose = [goalPose(1:2), continuousGoalTheta];
    if norm(path(end, 1:2) - goalPose(1:2)) ...
            <= numericalTolerance ...
            && abs(path(end, 3) - continuousGoalTheta) ...
                <= numericalTolerance
        path(end, :) = exactGoalPose;
    else
        path(end + 1, :) = exactGoalPose;
    end

    function value = heuristic(pose)
        translation = norm(goalPose(1:2) - pose(1:2));
        rotation = cornerRadius ...
            * abs(angle_difference(goalPose(3), pose(3)));
        value = hypot(translation, rotation);
    end

    function value = motion_cost(fromPose, toPose)
        translation = norm(toPose(1:2) - fromPose(1:2));
        rotation = cornerRadius ...
            * abs(angle_difference(toPose(3), fromPose(3)));
        value = hypot(translation, rotation);
    end

    function valid = transition_is_valid(fromPose, toPose)
        translationDistance = norm(toPose(1:2) - fromPose(1:2));
        thetaChange = angle_difference(toPose(3), fromPose(3));
        sampleCount = max(1, ceil(max( ...
            translationDistance / transitionXYResolution, ...
            abs(thetaChange) / transitionThetaResolution)));
        valid = true;
        for sampleIndex = 1:sampleCount
            fraction = sampleIndex / sampleCount;
            samplePose = [fromPose(1:2) ...
                + fraction * (toPose(1:2) - fromPose(1:2)), ...
                fromPose(3) + fraction * thetaChange];
            if ~configuration_is_valid(samplePose)
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

    function heap_push(nodeToPush, priorityToPush, gScoreToPush)
        if heapSize >= numel(heapNodes)
            newCapacity = 2 * numel(heapNodes);
            heapNodes(newCapacity, 1) = uint32(0);
            heapPriorities(newCapacity, 1) = inf;
            heapGScores(newCapacity, 1) = inf;
        end

        heapSize = heapSize + 1;
        insertionIndex = heapSize;
        while insertionIndex > 1
            parentIndex = floor(insertionIndex / 2);
            if heapPriorities(parentIndex) <= priorityToPush
                break;
            end
            heapNodes(insertionIndex) = heapNodes(parentIndex);
            heapPriorities(insertionIndex) = heapPriorities(parentIndex);
            heapGScores(insertionIndex) = heapGScores(parentIndex);
            insertionIndex = parentIndex;
        end
        heapNodes(insertionIndex) = nodeToPush;
        heapPriorities(insertionIndex) = priorityToPush;
        heapGScores(insertionIndex) = gScoreToPush;
    end

    function [poppedNode, poppedPriority, poppedGScore] = heap_pop()
        poppedNode = heapNodes(1);
        poppedPriority = heapPriorities(1);
        poppedGScore = heapGScores(1);
        lastNode = heapNodes(heapSize);
        lastPriority = heapPriorities(heapSize);
        lastGScore = heapGScores(heapSize);
        heapSize = heapSize - 1;
        if heapSize == 0
            return;
        end

        insertionIndex = 1;
        while true
            leftChild = 2 * insertionIndex;
            if leftChild > heapSize
                break;
            end
            rightChild = leftChild + 1;
            smallerChild = leftChild;
            if rightChild <= heapSize ...
                    && heapPriorities(rightChild) ...
                        < heapPriorities(leftChild)
                smallerChild = rightChild;
            end
            if heapPriorities(smallerChild) >= lastPriority
                break;
            end
            heapNodes(insertionIndex) = heapNodes(smallerChild);
            heapPriorities(insertionIndex) = heapPriorities(smallerChild);
            heapGScores(insertionIndex) = heapGScores(smallerChild);
            insertionIndex = smallerChild;
        end
        heapNodes(insertionIndex) = lastNode;
        heapPriorities(insertionIndex) = lastPriority;
        heapGScores(insertionIndex) = lastGScore;
    end
end

function pose = validate_pose(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) ...
            || ~isequal(size(value), [1, 3]) ...
            || any(~isfinite(value(:)))
        error('search_large_item_astar:InvalidPose', ...
              '%s必须是1x3有限实数向量。', argumentName);
    end
    pose = double(value);
end

function obstacles = normalise_obstacles(value)
    if isempty(value)
        obstacles = zeros(0, 2);
        return;
    end
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) ...
            || size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('search_large_item_astar:InvalidObstacles', ...
              'obstacles必须是有限实数N x 2矩阵或空数组。');
    end
    obstacles = double(value);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) ...
            || numel(value) ~= 2 || any(~isfinite(value(:)))
        error('search_large_item_astar:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = double(value(:).');
    if limits(1) >= limits(2)
        error('search_large_item_astar:InvalidLimits', ...
              '%s的下限必须小于上限。', argumentName);
    end
end

function angle = angle_difference(toAngle, fromAngle)
    angle = atan2(sin(toAngle - fromAngle), cos(toAngle - fromAngle));
end

function angle = wrap_angle(angle)
    angle = atan2(sin(angle), cos(angle));
end
