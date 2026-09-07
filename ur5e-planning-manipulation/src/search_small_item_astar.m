function [rawPath, expandedNodes] = search_small_item_astar( ...
        startPosition, goalPosition, obstacles, xLimits, yLimits, config)
%SEARCH_SMALL_ITEM_ASTAR 在二维网格中搜索小物体安全中心路径。
%   网格锚定精确起点，目标作为经过连续边验证的虚拟节点。小物体和
%   障碍物分别使用65 mm和50 mm圆形近似。

    if nargin < 6
        config = small_item_astar_config();
    end
    algorithm = config.algorithm;
    geometry = config.geometry;
    xyResolution = algorithm.xyResolution;
    transitionResolution = algorithm.transitionResolution;
    itemRadius = geometry.itemRadius;
    minimumCenterDistance = geometry.minimumCenterDistance;
    numericalTolerance = geometry.numericalTolerance;

    startPosition = validate_xy(startPosition, 'startPosition');
    goalPosition = validate_xy(goalPosition, 'goalPosition');
    obstacles = normalise_obstacles(obstacles);
    xLimits = validate_limits(xLimits, 'xLimits');
    yLimits = validate_limits(yLimits, 'yLimits');

    safeXLimits = xLimits + [itemRadius, -itemRadius];
    safeYLimits = yLimits + [itemRadius, -itemRadius];
    if safeXLimits(1) > safeXLimits(2) ...
            || safeYLimits(1) > safeYLimits(2)
        error('search_small_item_astar:WorkspaceTooSmall', ...
              '工作区无法容纳半径65 mm的小物体。');
    end
    if ~position_is_valid(startPosition)
        error('search_small_item_astar:InvalidStart', ...
              '起点违反65 mm边界或115 mm障碍物clearance。');
    end
    if ~position_is_valid(goalPosition)
        error('search_small_item_astar:InvalidGoal', ...
              '目标违反65 mm边界或115 mm障碍物clearance。');
    end

    xOffsetMinimum = ceil((safeXLimits(1) - startPosition(1)) ...
        / xyResolution - numericalTolerance);
    xOffsetMaximum = floor((safeXLimits(2) - startPosition(1)) ...
        / xyResolution + numericalTolerance);
    yOffsetMinimum = ceil((safeYLimits(1) - startPosition(2)) ...
        / xyResolution - numericalTolerance);
    yOffsetMaximum = floor((safeYLimits(2) - startPosition(2)) ...
        / xyResolution + numericalTolerance);

    xOffsets = xOffsetMinimum:xOffsetMaximum;
    yOffsets = yOffsetMinimum:yOffsetMaximum;
    xGrid = startPosition(1) + xyResolution * xOffsets;
    yGrid = startPosition(2) + xyResolution * yOffsets;
    startXIndex = find(xOffsets == 0, 1);
    startYIndex = find(yOffsets == 0, 1);
    if isempty(startXIndex) || isempty(startYIndex)
        error('search_small_item_astar:StartOutsideGrid', ...
              '起点不在收缩后的工作区网格中。');
    end

    gridSize = [numel(xGrid), numel(yGrid)];
    stateCount = prod(double(gridSize));
    if stateCount > double(intmax('uint32'))
        error('search_small_item_astar:GridTooLarge', ...
              '搜索网格超过uint32索引容量。');
    end

    gScore = inf(gridSize);
    parent = zeros(gridSize, 'uint32');
    closed = false(gridSize);
    startNode = uint32(sub2ind( ...
        gridSize, startXIndex, startYIndex));
    gScore(double(startNode)) = 0;

    [actionX, actionY] = ndgrid(-1:1, -1:1);
    actions = [actionX(:), actionY(:)];
    actions(all(actions == 0, 2), :) = [];

    heapCapacity = 4096;
    heapNodes = zeros(heapCapacity, 1, 'uint32');
    heapPriorities = inf(heapCapacity, 1);
    heapGScores = inf(heapCapacity, 1);
    heapSize = 0;
    heap_push(startNode, heuristic(startPosition), 0);

    bestGoalCost = inf;
    bestGoalParent = uint32(0);
    expandedNodes = 0;
    maximumGoalConnectorDistance = sqrt(2) * xyResolution;

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
        [currentXIndex, currentYIndex] = ...
            ind2sub(gridSize, currentLinearIndex);
        currentPosition = [ ...
            xGrid(currentXIndex), yGrid(currentYIndex)];

        positionToGoal = norm(goalPosition - currentPosition);
        if positionToGoal <= maximumGoalConnectorDistance ...
                + numericalTolerance ...
                && transition_is_valid(currentPosition, goalPosition)
            candidateGoalCost = gScore(currentLinearIndex) ...
                + positionToGoal;
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

            nextNode = uint32(sub2ind( ...
                gridSize, nextXIndex, nextYIndex));
            nextLinearIndex = double(nextNode);
            if closed(nextLinearIndex)
                continue;
            end

            nextPosition = [xGrid(nextXIndex), yGrid(nextYIndex)];
            if ~transition_is_valid(currentPosition, nextPosition)
                continue;
            end

            transitionCost = norm(nextPosition - currentPosition);
            tentativeGScore = gScore(currentLinearIndex) ...
                + transitionCost;
            if tentativeGScore + numericalTolerance ...
                    < gScore(nextLinearIndex)
                gScore(nextLinearIndex) = tentativeGScore;
                parent(nextLinearIndex) = currentNode;
                estimatedTotalCost = tentativeGScore ...
                    + heuristic(nextPosition);
                heap_push(nextNode, estimatedTotalCost, tentativeGScore);
            end
        end

        if expandedNodes >= stateCount
            break;
        end
    end

    if bestGoalParent == 0
        rawPath = zeros(0, 2);
        return;
    end

    nodePath = bestGoalParent;
    node = bestGoalParent;
    while node ~= startNode
        node = parent(double(node));
        if node == 0
            error('search_small_item_astar:BrokenParentChain', ...
                  'A*父节点路径重建失败。');
        end
        nodePath(end + 1, 1) = node; %#ok<AGROW>
    end
    nodePath = flipud(nodePath);

    rawPath = zeros(numel(nodePath), 2);
    for pathIndex = 1:numel(nodePath)
        [xIndex, yIndex] = ind2sub( ...
            gridSize, double(nodePath(pathIndex)));
        rawPath(pathIndex, :) = [xGrid(xIndex), yGrid(yIndex)];
    end
    rawPath(1, :) = startPosition;
    if norm(rawPath(end, :) - goalPosition) <= numericalTolerance
        rawPath(end, :) = goalPosition;
    else
        rawPath(end + 1, :) = goalPosition;
    end

    function value = heuristic(position)
        value = norm(goalPosition - position);
    end

    function valid = transition_is_valid(fromPosition, toPosition)
        if ~position_is_valid(fromPosition)
            valid = false;
            return;
        end
        distance = norm(toPosition - fromPosition);
        sampleCount = max(1, ceil(distance / transitionResolution));
        valid = true;
        for sampleIndex = 1:sampleCount
            fraction = sampleIndex / sampleCount;
            samplePosition = fromPosition ...
                + fraction * (toPosition - fromPosition);
            if ~position_is_valid(samplePosition)
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
            heapPriorities(insertionIndex) = ...
                heapPriorities(parentIndex);
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
            heapPriorities(insertionIndex) = ...
                heapPriorities(smallerChild);
            heapGScores(insertionIndex) = heapGScores(smallerChild);
            insertionIndex = smallerChild;
        end
        heapNodes(insertionIndex) = lastNode;
        heapPriorities(insertionIndex) = lastPriority;
        heapGScores(insertionIndex) = lastGScore;
    end
end

function xy = validate_xy(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value(:)))
        error('search_small_item_astar:InvalidXY', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    xy = reshape(double(value), 1, 2);
end

function limits = validate_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
            || any(~isfinite(value(:)))
        error('search_small_item_astar:InvalidLimits', ...
              '%s必须包含两个有限实数。', argumentName);
    end
    limits = sort(reshape(double(value), 1, 2));
    if limits(1) >= limits(2)
        error('search_small_item_astar:InvalidLimits', ...
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
        error('search_small_item_astar:InvalidObstacles', ...
              'obstacles必须包含有限实数坐标。');
    end
    if ismatrix(value) && size(value, 2) == 2
        obstacles = double(value);
    elseif size(value, 1) == 1 && size(value, 2) == 2
        obstacles = reshape(permute(double(value), [3, 2, 1]), [], 2);
    else
        error('search_small_item_astar:InvalidObstacles', ...
              'obstacles必须是N x 2或1 x 2 x N数组。');
    end
end
