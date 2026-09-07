function pose = aruco_marker_pose_2d(transform, corners, ids, markerID)
%ARUCO_MARKER_POSE_2D 计算指定ArUco marker在Base平面中的二维位姿。
%   pose.position为Nx2位置，pose.theta为Nx1朝向，pose.corners为
%   4x2xN角点。theta由marker P1指向P2的+X轴方向定义。

    if ~isnumeric(markerID) || ~isreal(markerID) || ...
            ~isscalar(markerID) || ~isfinite(markerID) || ...
            markerID ~= fix(markerID)
        error('aruco_marker_pose_2d:InvalidMarkerID', ...
              'markerID必须是有限整数标量。');
    end

    markerIndices = find(double(ids(:)) == markerID);
    if isempty(markerIndices)
        error('aruco_marker_pose_2d:MissingMarker', ...
              '未检测到ArUco marker ID%d。', markerID);
    end

    markerCount = numel(markerIndices);
    position = zeros(markerCount, 2);
    theta = zeros(markerCount, 1);
    cornersBase = zeros(4, 2, markerCount);

    for markerIndex = 1:markerCount
        imageCorners = corners(:, :, markerIndices(markerIndex));
        baseCorners = transformPointsForward(transform, imageCorners);

        cornersBase(:, :, markerIndex) = baseCorners;
        position(markerIndex, :) = aruco_marker_center(baseCorners);

        markerXAxis = baseCorners(2, :) - baseCorners(1, :);
        if norm(markerXAxis) <= 1e-12
            error('aruco_marker_pose_2d:DegenerateMarkerAxis', ...
                  'ArUco marker ID%d的P1和P2映射到同一点。', markerID);
        end
        theta(markerIndex) = atan2(markerXAxis(2), markerXAxis(1));
    end

    pose.position = position;
    pose.theta = theta;
    pose.corners = cornersBase;
end
