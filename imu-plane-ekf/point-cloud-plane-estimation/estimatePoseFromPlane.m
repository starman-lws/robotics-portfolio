function [rollPitchRad, normalDistance] = estimatePoseFromPlane(planeNormal, centroid)
%estimatePoseFromPlane 根据平面法向量估计横滚角、俯仰角及法向距离
%   [rollPitchRad, normalDistance] = estimatePoseFromPlane(planeNormal, centroid)
%   根据已在目标坐标系中表达的平面单位法向量，计算与该法向量一致的
%   Roll 和 Pitch。Yaw 绕法向量旋转，无法由单个平面观测独立确定。
%
%   输入：
%     planeNormal - 3 元平面法向量，可为行向量或列向量
%     centroid    - 3 元平面质心，可为行向量或列向量；若坐标单位为 m，
%                   输出距离同样为 m
%
%   输出：
%     rollPitchRad  - [roll; pitch]，单位为 rad
%     normalDistance - 坐标原点到平面的无符号法向距离
%
%   姿态采用 ZYX Euler 约定。法向量方向会影响姿态结果，因此调用前应
%   通过视点或其他先验统一法向量朝向。
%
%   作者：Wenshao Lyu

    arguments
        planeNormal double {mustBeReal, mustBeFinite}
        centroid double {mustBeReal, mustBeFinite}
    end

    if numel(planeNormal) ~= 3
        error('PlaneEstimation:InvalidNormalSize', ...
            'planeNormal must contain exactly three elements.');
    end
    if numel(centroid) ~= 3
        error('PlaneEstimation:InvalidCentroidSize', ...
            'centroid must contain exactly three elements.');
    end

    normal = planeNormal(:);
    normalNorm = norm(normal);
    if normalNorm <= eps(class(normal))
        error('PlaneEstimation:ZeroNormal', ...
            'planeNormal must have nonzero magnitude.');
    end
    normal = normal / normalNorm;

    % 对 asin 的输入进行限幅，避免浮点舍入产生复数结果。
    clampedNormalX = min(max(normal(1), -1), 1);
    rollRad = atan2(normal(2), normal(3));
    pitchRad = asin(-clampedNormalX);

    rollPitchRad = [rollRad; pitchRad];
    normalDistance = abs(dot(centroid(:), normal));
end
