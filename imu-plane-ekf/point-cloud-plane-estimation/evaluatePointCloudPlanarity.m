function [isPlanar, maxDistance, rmsDistance] = ...
    evaluatePointCloudPlanarity( ...
    points, planeNormal, centroid, distanceTolerance)
%EVALUATEPOINTCLOUDPLANARITY 根据点面残差判断点云是否近似共面。
%   [ISPLANAR, MAXDISTANCE, RMSDISTANCE] = EVALUATEPOINTCLOUDPLANARITY(
%   POINTS, NORMAL, CENTROID, TOLERANCE) 计算所有点到指定平面的距离。
%   最大绝对距离不超过 TOLERANCE 时，ISPLANAR 返回 true。
%
%   输入
%     points             N×3 有限点云坐标 [m]
%     planeNormal        三维平面法向量
%     centroid           三维平面参考点 [m]
%     distanceTolerance  非负最大允许点面距离 [m]
%
%   输出
%     isPlanar           是否满足最大距离门限
%     maxDistance        最大绝对点面距离 [m]
%     rmsDistance        点面距离 RMS [m]
%
%   作者：Wenshao Lyu

    narginchk(4, 4);

    validateattributes(points, {'double'}, ...
        {'2d', 'real', 'finite', 'nonempty'}, mfilename, 'points', 1);
    validateattributes(planeNormal, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'planeNormal', 2);
    validateattributes(centroid, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'centroid', 3);
    validateattributes(distanceTolerance, {'double'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'}, ...
        mfilename, 'distanceTolerance', 4);

    if size(points, 2) ~= 3
        error('PointCloudPlaneEstimation:InvalidPointCloudSize', ...
            'points must be an N-by-3 matrix.');
    end

    planeNormal = planeNormal(:);
    normalMagnitude = norm(planeNormal);
    if normalMagnitude <= 1e-12
        error('PointCloudPlaneEstimation:InvalidPlaneNormal', ...
            'planeNormal must have nonzero magnitude.');
    end
    planeNormal = planeNormal / normalMagnitude;
    centroid = centroid(:);

    signedDistances = (points - centroid.') * planeNormal;
    absoluteDistances = abs(signedDistances);
    maxDistance = max(absoluteDistances);
    rmsDistance = sqrt(mean(signedDistances .^ 2));
    isPlanar = maxDistance <= distanceTolerance;
end
