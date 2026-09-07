function selection = validate_aruco_detections(ids, corners, config)
%VALIDATE_ARUCO_DETECTIONS 验证平面场景所需marker并选择reference实例。
%   ID4和ID6允许重复并全部保留。reference ID0-3重复时选择图像
%   面积最大的实例；ID5和ID7必须各出现一次。

    if ~isnumeric(ids) || ~isreal(ids) || ~isvector(ids) || ...
            any(~isfinite(ids(:))) || any(ids(:) ~= fix(ids(:)))
        error('validate_aruco_detections:InvalidIDs', ...
              'ids必须是有限整数marker ID向量。');
    end

    markerCount = numel(ids);
    if ~isnumeric(corners) || ~isreal(corners) || ...
            size(corners, 1) ~= 4 || size(corners, 2) ~= 2 || ...
            size(corners, 3) ~= markerCount || ...
            any(~isfinite(corners(:)))
        error('validate_aruco_detections:InvalidCorners', ...
              'corners必须是与ids对应的4x2xN有限实数数组。');
    end

    ids = double(ids(:));
    missingIDs = config.requiredIDs(~ismember(config.requiredIDs, ids.'));
    if ~isempty(missingIDs)
        error('validate_aruco_detections:MissingMarkers', ...
              '未检测到必需的ArUco marker ID: %s。', ...
              strjoin(string(missingIDs), ', '));
    end

    for markerID = config.singleInstanceIDs
        markerIndices = find(ids == markerID);
        if numel(markerIndices) ~= 1
            error('validate_aruco_detections:DuplicateMarker', ...
                  'ArUco marker ID%d必须且只能检测到一个实例。', ...
                  markerID);
        end
    end

    selection.referenceIndices = zeros(1, numel(config.referenceIDs));
    for referenceIndex = 1:numel(config.referenceIDs)
        markerID = config.referenceIDs(referenceIndex);
        candidates = find(ids == markerID);
        selection.referenceIndices(referenceIndex) = ...
            select_largest_marker(candidates, corners);
    end

    selection.obstacleIndices = find(ids == 4).';
    selection.smallItemIndex = find(ids == 5, 1);
    selection.bigMarkerIndices = find(ids == 6).';
    selection.targetBoardIndex = find(ids == 7, 1);
end

function selectedIndex = select_largest_marker(candidates, corners)
    if isscalar(candidates)
        selectedIndex = candidates;
        return;
    end

    areas = zeros(numel(candidates), 1);
    for candidateIndex = 1:numel(candidates)
        markerCorners = corners(:, :, candidates(candidateIndex));
        areas(candidateIndex) = polyarea( ...
            markerCorners(:, 1), markerCorners(:, 2));
    end

    [~, largestIndex] = max(areas);
    selectedIndex = candidates(largestIndex);
end
