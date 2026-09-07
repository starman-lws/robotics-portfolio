function [planeNormal, centroid, diagnostics] = fitPlaneSVD(points, viewpoint)
%FITPLANESVD 使用 SVD 拟合三维点云的最小二乘平面。
%   [NORMAL, CENTROID, DIAGNOSTICS] = FITPLANESVD(POINTS, VIEWPOINT)
%   对 N×3 点云去中心化，并将最小奇异值对应的右奇异向量作为
%   平面单位法向量。法向量统一朝向 VIEWPOINT，以消除 SVD 的符号歧义。
%
%   输入
%     points       N×3 有限点云坐标，长度单位为 m
%     viewpoint    3×1 观察点位置，与 points 位于同一坐标系 [m]
%
%   输出
%     planeNormal  3×1 朝向 viewpoint 的单位平面法向量
%     centroid     3×1 点云质心 [m]
%     diagnostics  奇异值、点面残差和拟合质量诊断
%
%   diagnostics 字段
%     singularValues   三个降序奇异值
%     signedDistances  各点到拟合平面的有符号距离 [m]
%     rmsDistance      点面距离 RMS [m]
%     maxDistance      最大绝对点面距离 [m]
%     planarityRatio   最小奇异值与第二奇异值之比
%     pointCount       输入点数量
%     orientationDot   法向量与质心至观察点方向的正内积 [m]
%
%   作者：Wenshao Lyu

    narginchk(2, 2);

    validateattributes(points, {'double'}, ...
        {'2d', 'real', 'finite', 'nonempty'}, mfilename, 'points', 1);
    validateattributes(viewpoint, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'viewpoint', 2);

    if size(points, 2) ~= 3
        error('PointCloudPlaneEstimation:InvalidPointCloudSize', ...
            'points must be an N-by-3 matrix.');
    end
    if size(points, 1) < 3
        error('PointCloudPlaneEstimation:InsufficientPoints', ...
            'At least three points are required to fit a plane.');
    end

    viewpoint = viewpoint(:);
    centroid = mean(points, 1).';
    centeredPoints = points - centroid.';

    % 最小奇异值对应的右奇异向量是最小二乘平面的法向方向。
    [~, singularValueMatrix, rightSingularVectors] = ...
        svd(centeredPoints, 0);
    singularValues = diag(singularValueMatrix);

    scaleTolerance = max(size(centeredPoints)) * ...
        eps(max(singularValues(1), 1));
    if singularValues(2) <= scaleTolerance
        error('PointCloudPlaneEstimation:DegeneratePointCloud', ...
            'The point cloud must span two independent in-plane directions.');
    end

    planeNormal = rightSingularVectors(:, 3);
    planeNormal = planeNormal / norm(planeNormal);

    % 使用观察点所在半空间确定法向量方向。
    viewpointDirection = viewpoint - centroid;
    orientationDot = dot(planeNormal, viewpointDirection);
    orientationTolerance = 100 * eps(max(norm(viewpointDirection), 1));
    if abs(orientationDot) <= orientationTolerance
        error('PointCloudPlaneEstimation:AmbiguousNormalOrientation', ...
            'viewpoint must not lie in the fitted plane.');
    end
    if orientationDot < 0
        planeNormal = -planeNormal;
        orientationDot = -orientationDot;
    end

    signedDistances = centeredPoints * planeNormal;
    absoluteDistances = abs(signedDistances);
    diagnostics = struct( ...
        'singularValues', singularValues, ...
        'signedDistances', signedDistances, ...
        'rmsDistance', sqrt(mean(signedDistances .^ 2)), ...
        'maxDistance', max(absoluteDistances), ...
        'planarityRatio', singularValues(3) / singularValues(2), ...
        'pointCount', size(points, 1), ...
        'orientationDot', orientationDot);
end
