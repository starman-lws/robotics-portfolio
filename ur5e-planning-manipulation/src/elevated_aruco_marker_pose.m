function pose = elevated_aruco_marker_pose( ...
        transform, corners, ids, markerID, cameraBasePosition, planeHeight)
%ELEVATED_ARUCO_MARKER_POSE 定位与桌面平行且高度已知的ArUco marker。
%   先通过桌面Homography求每条角点射线的Z=0交点，再沿相机射线修正
%   到Z=planeHeight。位置使用修正角点的projective center；theta沿用
%   P1到P2方向，因为统一的正比例高度修正不会改变平面朝向。

    markerIndices = find(double(ids(:)) == markerID);
    if numel(markerIndices) ~= 1
        error('elevated_aruco_marker_pose:InvalidMarkerCount', ...
              'ArUco marker ID%d必须且只能有一个实例。', markerID);
    end

    imageCorners = corners(:, :, markerIndices);
    basePlaneCorners = transformPointsForward(transform, imageCorners);
    elevatedCorners = project_marker_to_elevated_plane( ...
        basePlaneCorners, cameraBasePosition, planeHeight);

    markerXAxis = basePlaneCorners(2, :) - basePlaneCorners(1, :);
    if norm(markerXAxis) <= 1e-12
        error('elevated_aruco_marker_pose:DegenerateMarkerAxis', ...
              'ArUco marker ID%d的P1和P2映射到同一点。', markerID);
    end

    pose.position = [ ...
        aruco_marker_center(elevatedCorners(:, 1:2)), planeHeight];
    pose.theta = atan2(markerXAxis(2), markerXAxis(1));
    pose.corners = elevatedCorners;
end
